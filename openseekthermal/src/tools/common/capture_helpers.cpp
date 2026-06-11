// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "capture_helpers.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <endian.h>
#include <istream>
#include <ostream>

namespace openseekthermal::tools
{

namespace
{

std::string trim( const std::string &s )
{
  const auto first = s.find_first_not_of( " \t\r\n" );
  if ( first == std::string::npos )
    return "";
  const auto last = s.find_last_not_of( " \t\r\n" );
  return s.substr( first, last - first + 1 );
}

} // namespace

bool pickDevice( const std::string &serial, const std::string &port, SeekDevice &out,
                 std::string &err )
{
  auto devices = listDevices();
  if ( devices.empty() ) {
    err = "No SeekThermal devices found.";
    return false;
  }
  if ( !serial.empty() ) {
    auto it = std::find_if( devices.begin(), devices.end(),
                            [&]( const SeekDevice &d ) { return d.serial == serial; } );
    if ( it == devices.end() ) {
      err = "Device with serial '" + serial + "' not found.";
      return false;
    }
    out = *it;
    return true;
  }
  if ( !port.empty() ) {
    auto it = std::find_if( devices.begin(), devices.end(),
                            [&]( const SeekDevice &d ) { return d.usb_port == port; } );
    if ( it == devices.end() ) {
      err = "Device with port '" + port + "' not found.";
      return false;
    }
    out = *it;
    return true;
  }
  out = devices[0];
  return true;
}

bool pickDeviceInteractive( SeekDevice &out, std::string &err, std::istream &in, std::ostream &os )
{
  auto devices = listDevices();
  if ( devices.empty() ) {
    err = "No SeekThermal devices found.";
    return false;
  }
  if ( devices.size() == 1 ) {
    out = devices[0];
    os << "Using device: " << devices[0] << "\n";
    return true;
  }
  os << "Available devices:\n";
  for ( size_t i = 0; i < devices.size(); ++i ) os << "  [" << i << "] " << devices[i] << "\n";
  const int idx = promptInt( in, os, "Select device index", 0 );
  if ( idx < 0 || idx >= static_cast<int>( devices.size() ) ) {
    err = "Invalid device index.";
    return false;
  }
  out = devices[static_cast<size_t>( idx )];
  return true;
}

bool warmup( SeekThermalCamera &cam, int frames, std::string &err )
{
  for ( int i = 0; i < frames; ++i ) {
    unsigned char *buf = nullptr;
    size_t buf_size = 0;
    if ( cam.grabFrame( &buf, buf_size, nullptr ) != GrabFrameResult::SUCCESS ) {
      delete[] buf;
      err = "Warmup grab failed.";
      return false;
    }
    delete[] buf;
  }
  return true;
}

bool accumulateAverage( SeekThermalCamera &cam, int width, int height, int frames,
                        std::vector<uint64_t> &sum, int &skipped_cal, const std::string &status_label,
                        std::ostream *progress, bool preview, std::string &err )
{
  const size_t pixel_count = static_cast<size_t>( width ) * height;
  sum.assign( pixel_count, 0 );
  skipped_cal = 0;
  int accumulated = 0;
  if ( preview && progress )
    *progress << "\033[s"; // anchor the in-place preview below the printed instructions
  while ( accumulated < frames ) {
    FrameHeader header;
    unsigned char *buf = nullptr;
    size_t buf_size = 0;
    // Raw counts (pre temperature-mapping), so the averaged frame is in the same
    // domain the vignette correction is applied in.
    const GrabFrameResult res = cam.grabRawCountsFrame( &buf, buf_size, &header );
    if ( res != GrabFrameResult::SUCCESS ) {
      delete[] buf;
      err = "Capture grab failed: " + to_string( res );
      return false;
    }
    // Skip non-thermal frames
    if ( header.getFrameType() != FrameType::THERMAL_FRAME ) {
      ++skipped_cal;
      delete[] buf;
      continue;
    }
    const auto *frame = reinterpret_cast<const uint16_t *>( buf );
    for ( size_t i = 0; i < pixel_count; ++i ) sum[i] += frame[i];
    ++accumulated;
    if ( preview && progress ) {
      renderFrameAnsi( *progress, frame, width, height, width, 48,
                       status_label + " " + std::to_string( accumulated ) + " / " +
                           std::to_string( frames ),
                       true );
    }
    delete[] buf;
    if ( !preview && progress && ( accumulated % 10 == 0 || accumulated == frames ) )
      *progress << "  captured " << accumulated << " / " << frames << "\n";
  }
  return true;
}

bool captureWelfordRaw( SeekThermalCamera &cam, const SeekDevice &device, int width, int height,
                        int frames, RawPixelStats &out, std::ostream *progress, bool preview,
                        std::string &err )
{
  const int header_bytes = device._getFrameHeaderSize();
  const int row_step_pixels = device._getRowStep() / 2;
  const int frame_type_offset = FrameHeader::GetFrameTypeOffset( device.type );
  const int transfer_total = device._getFrameTransferTotalSize();
  const size_t pixel_count = static_cast<size_t>( width ) * height;

  out.mean.assign( pixel_count, 0.0 );
  out.stddev.assign( pixel_count, 0.0 );
  out.valid_count.assign( pixel_count, 0 );
  out.sentinel_count.assign( pixel_count, 0 );
  std::vector<double> m2( pixel_count, 0.0 );

  int thermal = 0;
  long attempts = 0;
  const long max_attempts = static_cast<long>( frames ) * 20 + 400;
  if ( preview && progress )
    *progress << "\033[s"; // anchor the in-place preview below the printed instructions
  while ( thermal < frames ) {
    if ( ++attempts > max_attempts ) {
      err = "Gave up after " + std::to_string( attempts ) + " transfers with only " +
            std::to_string( thermal ) + "/" + std::to_string( frames ) + " THERMAL frames.";
      return false;
    }
    unsigned char *raw = nullptr;
    size_t raw_size = 0;
    GrabFrameResult res;
    try {
      res = cam._grabRawFrame( &raw, raw_size );
    } catch ( const std::exception &e ) {
      delete[] raw;
      err = std::string( "_grabRawFrame threw: " ) + e.what();
      return false;
    }
    if ( res != GrabFrameResult::SUCCESS || static_cast<int>( raw_size ) < transfer_total ) {
      delete[] raw;
      continue;
    }
    uint16_t raw_ft = 0;
    // Bound against the transfer size, not header_bytes: on the original Compact
    // header_bytes is 0 while the frame-type word sits at transfer offset 20, so a
    // header_bytes bound would never read it. raw_size >= transfer_total here.
    if ( frame_type_offset + 2 <= transfer_total ) {
      std::memcpy( &raw_ft, raw + frame_type_offset, 2 );
      raw_ft = le16toh( raw_ft );
    }
    if ( raw_ft != 3 /* THERMAL */ ) {
      delete[] raw;
      continue;
    }
    const auto *px = reinterpret_cast<const uint16_t *>( raw + header_bytes );
    for ( int y = 0; y < height; ++y ) {
      for ( int x = 0; x < width; ++x ) {
        const size_t i = static_cast<size_t>( y ) * width + x;
        const uint16_t v = le16toh( px[y * row_step_pixels + x] );
        if ( v == 0 || v == 0xffff ) {
          ++out.sentinel_count[i];
          continue;
        }
        ++out.valid_count[i];
        const double delta = static_cast<double>( v ) - out.mean[i];
        out.mean[i] += delta / static_cast<double>( out.valid_count[i] );
        const double delta2 = static_cast<double>( v ) - out.mean[i];
        m2[i] += delta * delta2;
      }
    }
    ++thermal;
    if ( preview && progress ) {
      renderFrameAnsi( *progress, px, width, height, row_step_pixels, 48,
                       "Pan the camera — capturing " + std::to_string( thermal ) + " / " +
                           std::to_string( frames ),
                       true );
    }
    delete[] raw;
    if ( !preview && progress && ( thermal % 10 == 0 || thermal == frames ) )
      *progress << "  captured " << thermal << " / " << frames << "\n";
  }
  out.n = thermal;
  for ( size_t i = 0; i < pixel_count; ++i ) {
    if ( out.valid_count[i] > 1 )
      out.stddev[i] = std::sqrt( m2[i] / static_cast<double>( out.valid_count[i] - 1 ) );
  }
  return true;
}

bool captureAnchorLive( SeekThermalCamera &cam, const SeekDevice &device, int rx, int ry, int rw,
                        int rh, double robust_percentile, bool is_hot, int frames,
                        TwoPointAnchor &out, std::ostream *progress, bool preview, std::string &err )
{
  const int header_bytes = device._getFrameHeaderSize();
  const int row_step_pixels = device._getRowStep() / 2;
  const int frame_type_offset = FrameHeader::GetFrameTypeOffset( device.type );
  const int transfer_total = device._getFrameTransferTotalSize();
  const int width = device.getFrameWidth();
  const int height = device.getFrameHeight();

  double sum_pct = 0.0;
  double sum_mean = 0.0;
  int kept = 0;
  int thermal = 0;
  // Safety bound so a misbehaving stream cannot hang the wizard. Non-thermal
  // transfers (shutter, calibration) are interleaved, hence the generous slack.
  long attempts = 0;
  const long max_attempts = static_cast<long>( frames ) * 20 + 400;
  if ( preview && progress )
    *progress << "\033[s"; // anchor the in-place preview below the printed instructions
  while ( thermal < frames ) {
    if ( ++attempts > max_attempts ) {
      err = "Gave up after " + std::to_string( attempts ) + " transfers with only " +
            std::to_string( thermal ) + "/" + std::to_string( frames ) + " usable THERMAL frames.";
      return false;
    }
    unsigned char *raw = nullptr;
    size_t raw_size = 0;
    GrabFrameResult res;
    try {
      res = cam._grabRawFrame( &raw, raw_size );
    } catch ( const std::exception &e ) {
      delete[] raw;
      err = std::string( "_grabRawFrame threw: " ) + e.what();
      return false;
    }
    if ( res != GrabFrameResult::SUCCESS || static_cast<int>( raw_size ) < transfer_total ) {
      delete[] raw;
      continue;
    }
    uint16_t raw_ft = 0;
    // Bound against the transfer size, not header_bytes: on the original Compact
    // header_bytes is 0 while the frame-type word sits at transfer offset 20, so a
    // header_bytes bound would never read it. raw_size >= transfer_total here.
    if ( frame_type_offset + 2 <= transfer_total ) {
      std::memcpy( &raw_ft, raw + frame_type_offset, 2 );
      raw_ft = le16toh( raw_ft );
    }
    if ( raw_ft != 3 /* THERMAL */ ) {
      delete[] raw;
      continue;
    }
    const auto *px = reinterpret_cast<const uint16_t *>( raw + header_bytes );
    if ( accumulateAnchorFrame( px, row_step_pixels, rx, ry, rw, rh, robust_percentile, is_hot,
                                sum_pct, sum_mean, kept ) ) {
      ++thermal;
      if ( preview && progress ) {
        renderFrameAnsi( *progress, px, width, height, row_step_pixels, 48,
                         "Hold steady — capturing " + std::to_string( thermal ) + " / " +
                             std::to_string( frames ),
                         true );
      } else if ( progress && ( thermal % 10 == 0 || thermal == frames ) ) {
        *progress << "  captured " << thermal << " / " << frames << "\n";
      }
    }
    delete[] raw;
  }
  if ( kept == 0 ) {
    err = "No usable THERMAL frames captured.";
    return false;
  }
  out = finalizeAnchor( sum_pct, sum_mean, kept, robust_percentile, is_hot );
  return true;
}

// --- prompt helpers ----------------------------------------------------------

std::string promptLine( std::istream &in, std::ostream &os, const std::string &label,
                        const std::string &def )
{
  os << label;
  if ( !def.empty() )
    os << " [" << def << "]";
  os << ": ";
  os.flush();
  std::string line;
  if ( !std::getline( in, line ) )
    return def;
  line = trim( line );
  return line.empty() ? def : line;
}

int promptInt( std::istream &in, std::ostream &os, const std::string &label, int def )
{
  while ( true ) {
    os << label << " [" << def << "]: ";
    os.flush();
    std::string line;
    if ( !std::getline( in, line ) )
      return def;
    line = trim( line );
    if ( line.empty() )
      return def;
    try {
      size_t pos = 0;
      const int v = std::stoi( line, &pos );
      if ( pos == line.size() )
        return v;
    } catch ( const std::exception & ) {
    }
    os << "  invalid integer, try again\n";
  }
}

double promptDouble( std::istream &in, std::ostream &os, const std::string &label, double def )
{
  while ( true ) {
    os << label << " [" << def << "]: ";
    os.flush();
    std::string line;
    if ( !std::getline( in, line ) )
      return def;
    line = trim( line );
    if ( line.empty() )
      return def;
    try {
      size_t pos = 0;
      const double v = std::stod( line, &pos );
      if ( pos == line.size() )
        return v;
    } catch ( const std::exception & ) {
    }
    os << "  invalid number, try again\n";
  }
}

bool promptYesNo( std::istream &in, std::ostream &os, const std::string &label, bool def )
{
  while ( true ) {
    os << label << ( def ? " [Y/n]: " : " [y/N]: " );
    os.flush();
    std::string line;
    if ( !std::getline( in, line ) )
      return def;
    line = trim( line );
    if ( line.empty() )
      return def;
    const char c = static_cast<char>( std::tolower( static_cast<unsigned char>( line[0] ) ) );
    if ( c == 'y' )
      return true;
    if ( c == 'n' )
      return false;
    os << "  please answer y or n\n";
  }
}

} // namespace openseekthermal::tools
