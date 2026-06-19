// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

// dump_camera_data: capture a shareable dump of a camera for investigating new
// units. It records every byte returned by the Nano 300 / Compact Pro setup
// sequence (factory pages, firmware-info blocks, chip ID, error code) and then
// the first N raw frame transfers (header + body, unprocessed). The original
// Seek Compact has no factory pages, so for it only the raw frames are dumped.
//
// Camera state is left untouched beyond what the production setup already does,
// plus, on the Compact Pro, a handful of GET_* probes after "orphan"
// SET_*_FEATURES writes that the stock setup leaves undrained:
//
//   * No RESET_DEVICE.
//   * No SET_FACTORY_SETTINGS / BEGIN_MEMORY_WRITE / WRITE_MEMORY_DATA /
//     COMPLETE_MEMORY_WRITE.
//   * Only the SET_* writes that the stock setup already performs (those are
//     required to put the camera into a state where the GET_* commands return
//     meaningful data) — plus extra GET_* probes after orphan SETs.
//
// File format:
//   * <out>.bin   — sequence of chunks. Each chunk is:
//                       4 bytes "CHNK" magic
//                       4 bytes LE  seq id
//                       4 bytes LE  payload length N
//                       N bytes     payload
//   * <out>.json  — UTF-8 JSON, describes every chunk in the bin file:
//                       { "device": {...}, "chunks": [ {...}, ... ] }
//
// Each chunk record has: seq, kind, command, feature (hex string of bytes
// written via SET_*_FEATURES immediately before the read, or null), length,
// offset_in_bin, plus kind-specific fields (factory_addr / feature_code /
// frame_type / etc.). Frame chunks carry the full raw transfer.

#include "openseekthermal/detail/cameras/seek_thermal_compact_pro.hpp"
#include "openseekthermal/detail/cameras/seek_thermal_nano_300.hpp"
#include "openseekthermal/detail/exceptions.hpp"
#include "openseekthermal/openseekthermal.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <endian.h>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace openseekthermal;

namespace
{

// ---------------------------------------------------------------------------
// Capture state shared between the Nano 300 setup override and the rest of
// main(). One instance per dump run.

struct ChunkRecord {
  uint32_t seq = 0;
  std::string kind;        // "setup_read" / "orphan_probe" / "extra_factory" / "frame"
  std::string command;     // SeekDeviceCommand name (e.g. "GET_FACTORY_SETTINGS")
  std::string feature_hex; // "" if no SET_*_FEATURES was issued for this read
  uint32_t length = 0;
  uint64_t offset_in_bin = 0;
  // Optional kind-specific metadata. Stored as raw strings/ints — emitted into
  // the JSON sidecar verbatim.
  std::string note;
  int factory_addr = -1; // -1 == not applicable
  int frame_type = -1;   // raw frame type word (le16), for frame chunks
  int frame_number = -1;
};

class DumpRecorder
{
public:
  DumpRecorder( const fs::path &bin_path ) : out_( bin_path, std::ios::binary )
  {
    if ( !out_ ) {
      throw std::runtime_error( "Failed to open output file '" + bin_path.string() + "'" );
    }
  }

  // Record an opaque payload. Returns the assigned sequence number.
  uint32_t addChunk( ChunkRecord meta, const unsigned char *data, size_t length )
  {
    meta.seq = next_seq_++;
    meta.length = static_cast<uint32_t>( length );
    meta.offset_in_bin = static_cast<uint64_t>( out_.tellp() );
    // Magic
    out_.write( "CHNK", 4 );
    const uint32_t seq_le = htole32( meta.seq );
    out_.write( reinterpret_cast<const char *>( &seq_le ), 4 );
    const uint32_t len_le = htole32( meta.length );
    out_.write( reinterpret_cast<const char *>( &len_le ), 4 );
    if ( length > 0 ) {
      out_.write( reinterpret_cast<const char *>( data ), static_cast<std::streamsize>( length ) );
    }
    chunks_.push_back( std::move( meta ) );
    return chunks_.back().seq;
  }

  const std::vector<ChunkRecord> &chunks() const { return chunks_; }

private:
  std::ofstream out_;
  uint32_t next_seq_ = 0;
  std::vector<ChunkRecord> chunks_;
};

// Tracks the most recent SET_*_FEATURES write so reads can be annotated with
// the "input" that produced them.
struct LastFeatureWrite {
  std::string command; // e.g. "SET_FACTORY_SETTINGS_FEATURES"
  std::vector<unsigned char> data;
  bool valid = false;
};

std::string toHex( const std::vector<unsigned char> &data )
{
  std::ostringstream ss;
  for ( size_t i = 0; i < data.size(); ++i ) {
    if ( i != 0 )
      ss << ' ';
    ss << std::hex << std::setfill( '0' ) << std::setw( 2 ) << static_cast<int>( data[i] );
  }
  return ss.str();
}

std::string jsonEscape( const std::string &s )
{
  std::ostringstream ss;
  for ( char c : s ) {
    switch ( c ) {
    case '"':
      ss << "\\\"";
      break;
    case '\\':
      ss << "\\\\";
      break;
    case '\n':
      ss << "\\n";
      break;
    case '\r':
      ss << "\\r";
      break;
    case '\t':
      ss << "\\t";
      break;
    default:
      if ( static_cast<unsigned char>( c ) < 0x20 ) {
        ss << "\\u" << std::hex << std::setfill( '0' ) << std::setw( 4 ) << static_cast<int>( c );
      } else {
        ss << c;
      }
    }
  }
  return ss.str();
}

// ---------------------------------------------------------------------------
// Free probe helpers. The Nano 300 and Compact Pro share the same factory /
// firmware-info command set, so the post-setup probes are identical for both
// devices. Subclasses expose read/write via lambdas that call the protected
// SeekThermalCamera::read/write members.

using WriteFn = std::function<bool( SeekDeviceCommand, const std::vector<unsigned char> & )>;
using ReadFn = std::function<bool( SeekDeviceCommand, std::vector<unsigned char> & )>;

// Probe additional factory pages beyond 0xA00 that the stock setup doesn't
// touch. Stops on the first all-zero or duplicate page.
size_t probeExtraFactoryPages( const WriteFn &write_fn, const ReadFn &read_fn,
                               DumpRecorder &recorder, int max_addr )
{
  size_t recorded = 0;
  std::vector<unsigned char> prev_payload;
  for ( int addr = 0xA00; addr < max_addr; addr += 0x20 ) {
    const uint16_t addr_le = htole16( addr );
    const auto *addr_bytes = reinterpret_cast<const unsigned char *>( &addr_le );
    std::vector<unsigned char> feature{ 0x20, 0x00, addr_bytes[0], addr_bytes[1], 0x00, 0x00 };
    if ( !write_fn( SeekDeviceCommand::SET_FACTORY_SETTINGS_FEATURES, feature ) ) {
      std::cerr << "extra-page probe: SET_FACTORY_SETTINGS_FEATURES failed at 0x" << std::hex
                << addr << std::dec << " — stopping\n";
      break;
    }
    std::vector<unsigned char> payload( 64 );
    try {
      if ( !read_fn( SeekDeviceCommand::GET_FACTORY_SETTINGS, payload ) ) {
        std::cerr << "extra-page probe: GET_FACTORY_SETTINGS short read at 0x" << std::hex << addr
                  << std::dec << " — stopping\n";
        break;
      }
    } catch ( const std::exception &e ) {
      std::cerr << "extra-page probe: error at 0x" << std::hex << addr << std::dec << ": "
                << e.what() << " — stopping\n";
      break;
    }
    bool all_zero =
        std::all_of( payload.begin(), payload.end(), []( unsigned char b ) { return b == 0; } );
    bool duplicate = ( !prev_payload.empty() && prev_payload == payload );
    ChunkRecord rec;
    rec.kind = "extra_factory";
    rec.command = "GET_FACTORY_SETTINGS";
    rec.feature_hex = toHex( feature );
    rec.factory_addr = addr;
    if ( all_zero )
      rec.note = "all_zero";
    else if ( duplicate )
      rec.note = "duplicate_of_previous";
    recorder.addChunk( rec, payload.data(), payload.size() );
    ++recorded;
    if ( all_zero ) {
      std::cerr << "extra-page probe: page 0x" << std::hex << addr << " is all zero, stopping\n";
      break;
    }
    if ( duplicate ) {
      std::cerr << "extra-page probe: page 0x" << std::hex << addr
                << " duplicates previous, stopping\n";
      break;
    }
    prev_payload = payload;
  }
  return recorded;
}

// ---------------------------------------------------------------------------
// Subclass SeekThermalNano300 so we can override setupCamera() and use the
// protected read/write helpers. The setup logic is a literal copy of the
// production sequence — see seek_thermal_nano_300.cpp::setupCamera() — with
// every read result recorded into the DumpRecorder. We must NOT skip the SET_*
// writes; the camera will not respond to the factory-page reads otherwise.

class DumpingNano300 : public SeekThermalNano300
{
public:
  DumpingNano300( const SeekDevice &device, DumpRecorder &recorder, libusb_context *ctx = nullptr )
      : SeekThermalNano300( device, ctx ), recorder_( recorder ), device_copy_( device )
  {
  }

  // Lambdas that expose the protected read/write to the free probe helpers.
  WriteFn writeFn()
  {
    return [this]( SeekDeviceCommand c, const std::vector<unsigned char> &d ) {
      return this->write( c, d );
    };
  }

  ReadFn readFn()
  {
    return
        [this]( SeekDeviceCommand c, std::vector<unsigned char> &d ) { return this->read( c, d ); };
  }

protected:
  // Literal copy of the stock setupCamera() with reads wrapped to record their
  // payloads. Do NOT diverge from the production order — the camera will refuse
  // subsequent reads otherwise. See seek_thermal_nano_300.cpp.
  void setupCamera() override
  {
    if ( !write( SeekDeviceCommand::TARGET_PLATFORM, { 0x00, 0x00 } ) ) {
      close();
      if ( !write( SeekDeviceCommand::TARGET_PLATFORM, { 0x00, 0x00 } ) ) {
        throw SeekSetupError( "Failed to set target platform!" );
      }
    }
    int retries = 0;
    std::vector<unsigned char> data( 2 );
    do {
      if ( ++retries > 10 )
        throw SeekSetupError( "Failed to set operation mode to off after 10 attempts!" );
      if ( !write( SeekDeviceCommand::SET_OPERATION_MODE, { 0x00, 0x00 } ) )
        throw SeekSetupError( "Failed to set operation mode!" );
      doRead( SeekDeviceCommand::GET_OPERATION_MODE, data, "setup_read",
              "initial GET_OPERATION_MODE poll" );
    } while ( data[0] != 0x00 || data[1] != 0x00 );

    if ( !write( SeekDeviceCommand::SET_IMAGE_PROCESSING_MODE, { 0x08, 0x00 } ) )
      throw SeekSetupError( "Failed to set image processing mode!" );
    setFeature( SeekDeviceCommand::SET_FACTORY_SETTINGS_FEATURES,
                { 0x08, 0x00, 0x02, 0x06, 0x00, 0x00 } );
    data.resize( 16 );
    doRead( SeekDeviceCommand::GET_FACTORY_SETTINGS, data, "setup_read", "feature 08 00 02 06 (1/2)" );
    data.resize( 4 );
    clearFeature();
    doRead( SeekDeviceCommand::GET_FIRMWARE_INFO, data, "setup_read", "short firmware info (1)" );
    setFeature( SeekDeviceCommand::SET_FACTORY_SETTINGS_FEATURES,
                { 0x08, 0x00, 0x02, 0x06, 0x00, 0x00 } );
    data.resize( 16 );
    doRead( SeekDeviceCommand::GET_FACTORY_SETTINGS, data, "setup_read", "feature 08 00 02 06 (2/2)" );
    setFeature( SeekDeviceCommand::SET_FIRMWARE_INFO_FEATURES, { 0x17, 0x00 } );
    data.resize( 64 );
    doRead( SeekDeviceCommand::GET_FIRMWARE_INFO, data, "setup_read", "firmware info feature 0x17" );
    data.resize( 4 );
    clearFeature();
    doRead( SeekDeviceCommand::GET_FIRMWARE_INFO, data, "setup_read", "short firmware info (2)" );
    data.resize( 12 );
    doRead( SeekDeviceCommand::READ_CHIP_ID, data, "setup_read", "chip id (1)" );
    setFeature( SeekDeviceCommand::SET_FIRMWARE_INFO_FEATURES, { 0x15, 0x00 } );
    data.resize( 64 );
    doRead( SeekDeviceCommand::GET_FIRMWARE_INFO, data, "setup_read", "firmware info feature 0x15" );
    data.resize( 12 );
    clearFeature();
    doRead( SeekDeviceCommand::READ_CHIP_ID, data, "setup_read", "chip id (2)" );
    data.resize( 4 );
    doRead( SeekDeviceCommand::GET_FIRMWARE_INFO, data, "setup_read", "short firmware info (3)" );

    for ( int addr = 0; addr < 0xA00; addr += 0x20 ) {
      const uint16_t addr_le = htole16( addr );
      const auto *addr_bytes = reinterpret_cast<const unsigned char *>( &addr_le );
      std::vector<unsigned char> feature{ 0x20, 0x00, addr_bytes[0], addr_bytes[1], 0x00, 0x00 };
      setFeature( SeekDeviceCommand::SET_FACTORY_SETTINGS_FEATURES, feature );
      data.resize( 64 );
      ChunkRecord rec;
      rec.kind = "setup_read";
      rec.command = "GET_FACTORY_SETTINGS";
      rec.feature_hex = toHex( feature );
      rec.factory_addr = addr;
      doReadWithRecord( SeekDeviceCommand::GET_FACTORY_SETTINGS, data, std::move( rec ) );
    }

    setFeature( SeekDeviceCommand::SET_FIRMWARE_INFO_FEATURES, { 0x15, 0x00 } );
    data.resize( 64 );
    doRead( SeekDeviceCommand::GET_FIRMWARE_INFO, data, "setup_read",
            "firmware info feature 0x15 (final)" );
    data.resize( 4 );
    clearFeature();
    doRead( SeekDeviceCommand::GET_ERROR_CODE, data, "setup_read", "final error code" );
    if ( data[0] != 0x00 && data[1] != 0x00 && data[2] != 0x00 && data[3] != 0x00 )
      throw SeekSetupError( "Camera reported error code during setup" );
    if ( !write( SeekDeviceCommand::TOGGLE_SHUTTER, { 0xFC, 0x00, 0x04, 0x00 } ) )
      throw SeekSetupError( "Failed to toggle shutter!" );
    retries = 0;
    do {
      if ( ++retries > 10 )
        throw SeekSetupError( "Failed to set operation mode to on after 10 attempts!" );
      if ( !write( SeekDeviceCommand::SET_OPERATION_MODE, { 0x01, 0x00 } ) ) {
        data.resize( 4 );
        doRead( SeekDeviceCommand::GET_ERROR_CODE, data, "setup_read",
                "error code after failed SET_OPERATION_MODE on" );
        throw SeekSetupError( "Failed to set operation mode to on!" );
      }
      data.resize( 2 );
      doRead( SeekDeviceCommand::GET_OPERATION_MODE, data, "setup_read", "operation mode poll on" );
    } while ( data[0] != 0x01 || data[1] != 0x00 );
  }

private:
  // Issue a SET_*_FEATURES write and remember the bytes so the next read can
  // record them.
  void setFeature( SeekDeviceCommand command, std::vector<unsigned char> feature )
  {
    if ( !write( command, feature ) )
      throw SeekSetupError( "Failed to set features for " + to_string( command ) );
    last_feature_.command = to_string( command );
    last_feature_.data = std::move( feature );
    last_feature_.valid = true;
  }

  void clearFeature() { last_feature_.valid = false; }

  // Plain read + record (no kind-specific metadata).
  void doRead( SeekDeviceCommand command, std::vector<unsigned char> &data, const std::string &kind,
               const std::string &note )
  {
    if ( !read( command, data ) )
      throw SeekSetupError( "Short read for " + to_string( command ) );
    ChunkRecord rec;
    rec.kind = kind;
    rec.command = to_string( command );
    rec.note = note;
    if ( last_feature_.valid )
      rec.feature_hex = toHex( last_feature_.data );
    recorder_.addChunk( rec, data.data(), data.size() );
  }

  // Read + record with caller-supplied metadata (kind/feature/factory_addr).
  void doReadWithRecord( SeekDeviceCommand command, std::vector<unsigned char> &data, ChunkRecord rec )
  {
    if ( !read( command, data ) )
      throw SeekSetupError( "Short read for " + to_string( command ) );
    recorder_.addChunk( std::move( rec ), data.data(), data.size() );
  }

  DumpRecorder &recorder_;
  SeekDevice device_copy_;
  LastFeatureWrite last_feature_;
};

// ---------------------------------------------------------------------------
// Compact Pro capture. Mirrors the production setup in
// seek_thermal_compact_pro.cpp::setupCamera() but inserts a GET_* read after
// each "orphan" SET_*_FEATURES that the stock setup leaves undrained. Those
// reads are tagged kind = "orphan_probe" in the sidecar — they're educated
// guesses at calibration data the production code never bothered to retrieve.

class DumpingCompactPro : public SeekThermalCompactPro
{
public:
  DumpingCompactPro( const SeekDevice &device, DumpRecorder &recorder, libusb_context *ctx = nullptr )
      : SeekThermalCompactPro( device, ctx ), recorder_( recorder ), device_copy_( device )
  {
  }

  WriteFn writeFn()
  {
    return [this]( SeekDeviceCommand c, const std::vector<unsigned char> &d ) {
      return this->write( c, d );
    };
  }

  ReadFn readFn()
  {
    return
        [this]( SeekDeviceCommand c, std::vector<unsigned char> &d ) { return this->read( c, d ); };
  }

protected:
  void setupCamera() override
  {
    if ( !write( SeekDeviceCommand::TARGET_PLATFORM, { 0x01 } ) ) {
      close();
      openDeviceForRetry();
      if ( !write( SeekDeviceCommand::TARGET_PLATFORM, { 0x01 } ) ) {
        throw SeekSetupError( "Failed to set target platform!" );
      }
    }
    int retries = 0;
    std::vector<unsigned char> data( 2 );
    do {
      if ( ++retries > 10 )
        throw SeekSetupError( "Failed to set operation mode to off after 10 attempts!" );
      if ( !write( SeekDeviceCommand::SET_OPERATION_MODE, { 0x00, 0x00 } ) )
        throw SeekSetupError( "Failed to set operation mode to off!" );
      doRead( SeekDeviceCommand::GET_OPERATION_MODE, data, "setup_read",
              "initial GET_OPERATION_MODE poll" );
    } while ( data[0] != 0x00 || data[1] != 0x00 );

    // Orphan #1: SET_FACTORY_SETTINGS_FEATURES { 0x06, 0x00, 0x08, ... }.
    // Stock setup never reads anything back after this. Insert a 64-byte
    // GET_FACTORY_SETTINGS probe to see what (if anything) the camera staged.
    setFeature( SeekDeviceCommand::SET_FACTORY_SETTINGS_FEATURES,
                { 0x06, 0x00, 0x08, 0x00, 0x00, 0x00 } );
    orphanProbe( SeekDeviceCommand::GET_FACTORY_SETTINGS, 64,
                 "orphan SET_FACTORY_SETTINGS_FEATURES feature 0x06 sub 0x08" );

    // Orphan #2: SET_FIRMWARE_INFO_FEATURES { 0x17, 0x00 }. Drain via
    // GET_FIRMWARE_INFO. (The Nano 300 setup reads 64 bytes after the same
    // feature-0x17 SET; assume the Compact Pro responds the same way.)
    setFeature( SeekDeviceCommand::SET_FIRMWARE_INFO_FEATURES, { 0x17, 0x00 } );
    orphanProbe( SeekDeviceCommand::GET_FIRMWARE_INFO, 64,
                 "orphan SET_FIRMWARE_INFO_FEATURES feature 0x17" );

    // Orphan #3: SET_FACTORY_SETTINGS_FEATURES { 0x01, 0x00, 0x00, 0x06, ... }.
    setFeature( SeekDeviceCommand::SET_FACTORY_SETTINGS_FEATURES,
                { 0x01, 0x00, 0x00, 0x06, 0x00, 0x00 } );
    orphanProbe( SeekDeviceCommand::GET_FACTORY_SETTINGS, 64,
                 "orphan SET_FACTORY_SETTINGS_FEATURES feature 0x01 sub 0" );

    // Orphan #4: SET_FACTORY_SETTINGS_FEATURES { 0x01, 0x00, 0x01, 0x06, ... }.
    setFeature( SeekDeviceCommand::SET_FACTORY_SETTINGS_FEATURES,
                { 0x01, 0x00, 0x01, 0x06, 0x00, 0x00 } );
    orphanProbe( SeekDeviceCommand::GET_FACTORY_SETTINGS, 64,
                 "orphan SET_FACTORY_SETTINGS_FEATURES feature 0x01 sub 1" );

    // Big factory-page loop, identical to the production sequence.
    for ( int addr = 0; addr < 0xA00; addr += 0x20 ) {
      const uint16_t addr_le = htole16( addr );
      const auto *addr_bytes = reinterpret_cast<const unsigned char *>( &addr_le );
      std::vector<unsigned char> feature{ 0x20, 0x00, addr_bytes[0], addr_bytes[1], 0x00, 0x00 };
      setFeature( SeekDeviceCommand::SET_FACTORY_SETTINGS_FEATURES, feature );
      data.resize( 64 );
      ChunkRecord rec;
      rec.kind = "setup_read";
      rec.command = "GET_FACTORY_SETTINGS";
      rec.feature_hex = toHex( feature );
      rec.factory_addr = addr;
      doReadWithRecord( SeekDeviceCommand::GET_FACTORY_SETTINGS, data, std::move( rec ) );
    }

    // Orphan #5: SET_FIRMWARE_INFO_FEATURES { 0x15, 0x00 } before
    // GET_ERROR_CODE. The Nano 300 reads 64 bytes after the same feature.
    setFeature( SeekDeviceCommand::SET_FIRMWARE_INFO_FEATURES, { 0x15, 0x00 } );
    orphanProbe( SeekDeviceCommand::GET_FIRMWARE_INFO, 64,
                 "orphan SET_FIRMWARE_INFO_FEATURES feature 0x15 (pre-error-code)" );

    clearFeature();
    data.resize( 4 );
    doRead( SeekDeviceCommand::GET_ERROR_CODE, data, "setup_read", "final error code" );
    if ( data[0] != 0x00 || data[1] != 0x00 || data[2] != 0x00 || data[3] != 0x00 )
      throw SeekSetupError( "Camera reported error code during setup" );

    if ( !write( SeekDeviceCommand::SET_IMAGE_PROCESSING_MODE, { 0x08, 0x00 } ) )
      throw SeekSetupError( "Failed to set image processing mode!" );
    retries = 0;
    data.resize( 2 );
    doRead( SeekDeviceCommand::GET_OPERATION_MODE, data, "setup_read",
            "operation mode poll on (initial)" );
    do {
      if ( ++retries > 10 )
        throw SeekSetupError( "Failed to set operation mode to on after 10 attempts!" );
      if ( !write( SeekDeviceCommand::SET_OPERATION_MODE, { 0x01, 0x00 } ) ) {
        data.resize( 4 );
        doRead( SeekDeviceCommand::GET_ERROR_CODE, data, "setup_read",
                "error code after failed SET_OPERATION_MODE on" );
        throw SeekSetupError( "Failed to set operation mode to on!" );
      }
      data.resize( 2 );
      doRead( SeekDeviceCommand::GET_OPERATION_MODE, data, "setup_read", "operation mode poll on" );
    } while ( data[0] != 0x01 || data[1] != 0x00 );
  }

private:
  // The base SeekThermalCamera does not expose openDevice() publicly — but it
  // is protected, so we can call it from this subclass after a close().
  void openDeviceForRetry() { openDevice(); }

  void setFeature( SeekDeviceCommand command, std::vector<unsigned char> feature )
  {
    if ( !write( command, feature ) )
      throw SeekSetupError( "Failed to set features for " + to_string( command ) );
    last_feature_.command = to_string( command );
    last_feature_.data = std::move( feature );
    last_feature_.valid = true;
  }

  void clearFeature() { last_feature_.valid = false; }

  void doRead( SeekDeviceCommand command, std::vector<unsigned char> &data, const std::string &kind,
               const std::string &note )
  {
    if ( !read( command, data ) )
      throw SeekSetupError( "Short read for " + to_string( command ) );
    ChunkRecord rec;
    rec.kind = kind;
    rec.command = to_string( command );
    rec.note = note;
    if ( last_feature_.valid )
      rec.feature_hex = toHex( last_feature_.data );
    recorder_.addChunk( rec, data.data(), data.size() );
  }

  void doReadWithRecord( SeekDeviceCommand command, std::vector<unsigned char> &data, ChunkRecord rec )
  {
    if ( !read( command, data ) )
      throw SeekSetupError( "Short read for " + to_string( command ) );
    recorder_.addChunk( std::move( rec ), data.data(), data.size() );
  }

  // Issue a GET_* of `size` bytes immediately after a SET_*_FEATURES that the
  // stock setup leaves undrained. Records the result (or a placeholder note on
  // failure) and does NOT throw — we want the rest of the setup to continue
  // even if the camera doesn't actually have anything staged for this feature.
  void orphanProbe( SeekDeviceCommand command, size_t size, const std::string &note )
  {
    std::vector<unsigned char> payload( size );
    ChunkRecord rec;
    rec.kind = "orphan_probe";
    rec.command = to_string( command );
    rec.note = note;
    if ( last_feature_.valid )
      rec.feature_hex = toHex( last_feature_.data );
    bool ok = false;
    std::string err;
    try {
      ok = read( command, payload );
    } catch ( const std::exception &e ) {
      err = e.what();
    }
    if ( !ok ) {
      // Record a zero-length chunk so the failure is still visible in the
      // sidecar. Append the error reason to the note.
      rec.note += err.empty() ? "; read failed (short or false)" : "; read failed: " + err;
      recorder_.addChunk( rec, nullptr, 0 );
      return;
    }
    recorder_.addChunk( rec, payload.data(), payload.size() );
  }

  DumpRecorder &recorder_;
  SeekDevice device_copy_;
  LastFeatureWrite last_feature_;
};

// ---------------------------------------------------------------------------
// JSON sidecar writer. Tiny hand-rolled emitter — no third-party JSON dep in
// the rest of the repo, and the schema is small enough that the cost of a
// dependency isn't justified.

void writeSidecar( const fs::path &json_path, const SeekDevice &device,
                   const std::vector<ChunkRecord> &chunks, const fs::path &bin_path )
{
  std::ofstream out( json_path );
  if ( !out )
    throw std::runtime_error( "Failed to open sidecar '" + json_path.string() + "'" );
  out << "{\n";
  out << "  \"format\": \"openseekthermal.dump_camera_data/v1\",\n";
  out << "  \"bin_file\": \"" << jsonEscape( bin_path.filename().string() ) << "\",\n";
  out << "  \"chunk_header\": {\n";
  out << "    \"magic\": \"CHNK\",\n";
  out << "    \"layout\": [\"u32_seq_le\", \"u32_length_le\"],\n";
  out << "    \"total_header_bytes\": 12\n";
  out << "  },\n";
  out << "  \"device\": {\n";
  out << "    \"type\": \"" << jsonEscape( to_string( device.type ) ) << "\",\n";
  out << "    \"serial\": \"" << jsonEscape( device.serial ) << "\",\n";
  out << "    \"usb_port\": \"" << jsonEscape( device.usb_port ) << "\",\n";
  out << "    \"frame_width\": " << device.getFrameWidth() << ",\n";
  out << "    \"frame_height\": " << device.getFrameHeight() << ",\n";
  out << "    \"frame_header_size\": " << device._getFrameHeaderSize() << "\n";
  out << "  },\n";
  out << "  \"chunks\": [\n";
  for ( size_t i = 0; i < chunks.size(); ++i ) {
    const auto &c = chunks[i];
    out << "    { \"seq\": " << c.seq;
    out << ", \"kind\": \"" << jsonEscape( c.kind ) << "\"";
    out << ", \"command\": \"" << jsonEscape( c.command ) << "\"";
    out << ", \"feature\": ";
    if ( c.feature_hex.empty() )
      out << "null";
    else
      out << "\"" << jsonEscape( c.feature_hex ) << "\"";
    out << ", \"length\": " << c.length;
    out << ", \"offset_in_bin\": " << c.offset_in_bin;
    if ( c.factory_addr >= 0 )
      out << ", \"factory_addr\": " << c.factory_addr;
    if ( c.frame_type >= 0 )
      out << ", \"frame_type\": " << c.frame_type;
    if ( c.frame_number >= 0 )
      out << ", \"frame_number\": " << c.frame_number;
    if ( !c.note.empty() )
      out << ", \"note\": \"" << jsonEscape( c.note ) << "\"";
    out << " }";
    if ( i + 1 < chunks.size() )
      out << ",";
    out << "\n";
  }
  out << "  ]\n";
  out << "}\n";
  if ( !out )
    throw std::runtime_error( "Failed to write sidecar" );
}

void printUsage( const char *argv0 )
{
  std::cout << "Usage: " << argv0 << " [options]\n"
            << "  --out PATH               output binary (default camera_data.bin).\n"
            << "                             a sibling .json sidecar with the same stem\n"
            << "                             is written alongside it.\n"
            << "  --serial S | --port P    select a specific device\n"
            << "  --frames N               number of raw frames to capture (default 100)\n"
            << "  --max-extra-addr H       enable probing of extra factory pages beyond\n"
            << "                             0xA00 up to this bound (hex, exclusive);\n"
            << "                             disabled by default\n"
            << "  -h | --help              show this message\n"
            << "\n"
            << "Captures the Nano 300 / Compact Pro setup sequence (factory pages,\n"
            << "firmware info blocks, chip ID, error code) plus the first N raw frame\n"
            << "transfers (header + body, unprocessed). The original Seek Compact has no\n"
            << "factory pages, so only the raw frames are dumped for it. Beyond the SET_*\n"
            << "writes the stock setup already performs (plus GET_* probes after orphan\n"
            << "SET_*_FEATURES writes in the Compact Pro path, tagged orphan_probe in the\n"
            << "sidecar), the device is left untouched: no RESET_DEVICE, no memory-write\n"
            << "commands, no SET_FACTORY_SETTINGS.\n";
}

} // namespace

int main( int argc, char **argv )
{
  fs::path out_path = "camera_data.bin";
  std::string serial;
  std::string port;
  int frame_count = 100;
  int max_extra_addr = 0; // extra-page probing disabled unless --max-extra-addr is given

  for ( int i = 1; i < argc; ++i ) {
    std::string arg = argv[i];
    if ( ( arg == "--out" || arg == "-o" ) && i + 1 < argc ) {
      out_path = argv[++i];
    } else if ( arg == "--serial" && i + 1 < argc ) {
      serial = argv[++i];
    } else if ( arg == "--port" && i + 1 < argc ) {
      port = argv[++i];
    } else if ( arg == "--frames" && i + 1 < argc ) {
      frame_count = std::max( 0, std::atoi( argv[++i] ) );
    } else if ( arg == "--max-extra-addr" && i + 1 < argc ) {
      max_extra_addr = static_cast<int>( std::strtol( argv[++i], nullptr, 0 ) );
    } else if ( arg == "-h" || arg == "--help" ) {
      printUsage( argv[0] );
      return 0;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n\n";
      printUsage( argv[0] );
      return 2;
    }
  }

  auto devices = listDevices();
  if ( devices.empty() ) {
    std::cerr << "No SeekThermal devices found.\n";
    return 1;
  }

  SeekDevice device = devices[0];
  if ( !serial.empty() ) {
    auto it = std::find_if( devices.begin(), devices.end(),
                            [&]( const SeekDevice &d ) { return d.serial == serial; } );
    if ( it == devices.end() ) {
      std::cerr << "Device with serial '" << serial << "' not found.\n";
      return 1;
    }
    device = *it;
  } else if ( !port.empty() ) {
    auto it = std::find_if( devices.begin(), devices.end(),
                            [&]( const SeekDevice &d ) { return d.usb_port == port; } );
    if ( it == devices.end() ) {
      std::cerr << "Device with port '" << port << "' not found.\n";
      return 1;
    }
    device = *it;
  }

  fs::path json_path = out_path;
  json_path.replace_extension( ".json" );

  std::cout << "Dumping camera data from " << device << "\n";
  std::cout << "  binary : " << out_path.string() << "\n";
  std::cout << "  sidecar: " << json_path.string() << "\n";

  std::unique_ptr<DumpRecorder> recorder;
  try {
    recorder = std::make_unique<DumpRecorder>( out_path );
  } catch ( const std::exception &e ) {
    std::cerr << e.what() << "\n";
    return 1;
  }

  // Nano 300 and Compact Pro expose factory pages during setup; the Dumping*
  // subclasses record every setup read. Other types (e.g. the original Seek
  // Compact) have no factory pages, so fall back to the production camera and
  // dump raw frames only.
  std::shared_ptr<SeekThermalCamera> camera;
  WriteFn write_fn;
  ReadFn read_fn;
  bool records_factory = true;
  if ( device.type == SeekDevice::Type::SeekThermalNano300 ) {
    auto cam = std::make_shared<DumpingNano300>( device, *recorder );
    write_fn = cam->writeFn();
    read_fn = cam->readFn();
    camera = cam;
  } else if ( device.type == SeekDevice::Type::SeekThermalCompactPro ) {
    auto cam = std::make_shared<DumpingCompactPro>( device, *recorder );
    write_fn = cam->writeFn();
    read_fn = cam->readFn();
    camera = cam;
  } else {
    std::cout << "Note: " << to_string( device.type )
              << " has no factory pages; capturing raw frames only.\n";
    camera = createCamera( device );
    records_factory = false;
  }

  try {
    camera->open();
  } catch ( const std::exception &e ) {
    std::cerr << "Failed to open camera (setup capture aborted): " << e.what() << "\n";
    // Still write whatever sidecar we have so far so the user can see how far
    // we got.
    try {
      writeSidecar( json_path, device, recorder->chunks(), out_path );
    } catch ( ... ) {
    }
    return 1;
  }
  if ( records_factory )
    std::cout << "Captured " << recorder->chunks().size() << " setup chunks.\n";

  // Capture the first N raw frame transfers in full (header + body, no
  // processing) so the static one-time startup frames are preserved. Uses
  // _grabRawFrame to get the unmodified bytes; the cooked grabFrame() would
  // shorten the header and apply corrections.
  std::cout << "Capturing " << frame_count << " raw frames ...\n";
  size_t frames_captured = 0;
  for ( int i = 0; i < frame_count; ++i ) {
    unsigned char *buf = nullptr;
    size_t buf_size = 0;
    GrabFrameResult res = camera->_grabRawFrame( &buf, buf_size );
    if ( res != GrabFrameResult::SUCCESS ) {
      std::cerr << "frame " << i << ": grab failed (" << to_string( res ) << "); stopping\n";
      delete[] buf;
      break;
    }
    // Pull the frame type / number out of the header so the analyzer can sort
    // by category without having to know the device-specific offsets.
    int frame_type_offset = FrameHeader::GetFrameTypeOffset( device.type );
    int frame_number_offset = FrameHeader::GetFrameNumberOffset( device.type );
    int frame_type = -1;
    int frame_number = -1;
    if ( frame_type_offset + 2 <= static_cast<int>( buf_size ) ) {
      uint16_t v;
      std::memcpy( &v, buf + frame_type_offset, 2 );
      frame_type = le16toh( v );
    }
    if ( frame_number_offset + 2 <= static_cast<int>( buf_size ) ) {
      uint16_t v;
      std::memcpy( &v, buf + frame_number_offset, 2 );
      frame_number = le16toh( v );
    }
    ChunkRecord rec;
    rec.kind = "frame";
    rec.command = "START_GET_IMAGE_TRANSFER";
    rec.frame_type = frame_type;
    rec.frame_number = frame_number;
    rec.note = "full raw frame transfer (header + body)";
    recorder->addChunk( rec, buf, buf_size );
    delete[] buf;
    ++frames_captured;
  }
  std::cout << "Captured " << frames_captured << " raw frames.\n";

  // Optional: probe extra factory pages beyond the stock 0xA00 (opt-in via
  // --max-extra-addr; Nano 300 / Compact Pro only).
  if ( records_factory && max_extra_addr > 0xA00 ) {
    std::cout << "Probing extra factory pages 0xA00 .. 0x" << std::hex << max_extra_addr << std::dec
              << " ...\n";
    size_t extra_pages = 0;
    try {
      extra_pages = probeExtraFactoryPages( write_fn, read_fn, *recorder, max_extra_addr );
    } catch ( const std::exception &e ) {
      std::cerr << "extra-page probe aborted: " << e.what() << "\n";
    }
    std::cout << "Recorded " << extra_pages << " extra factory pages beyond the stock 0xA00.\n";
  }

  try {
    camera->close();
  } catch ( ... ) {
  }

  try {
    writeSidecar( json_path, device, recorder->chunks(), out_path );
  } catch ( const std::exception &e ) {
    std::cerr << "Failed to write sidecar: " << e.what() << "\n";
    return 1;
  }
  std::cout << "Wrote " << recorder->chunks().size() << " chunks total to " << out_path.string()
            << " + " << json_path.string() << "\n";
  return 0;
}
