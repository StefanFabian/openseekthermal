// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "openseekthermal/detail/cameras/seek_thermal_camera.hpp"
#include "openseekthermal/detail/exceptions.hpp"
#include <libusb-1.0/libusb.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "../helpers.hpp"
#include "../logging.hpp"
#include "../timer.hpp"

namespace openseekthermal
{

SeekThermalCamera::SeekThermalCamera( SeekDevice device, libusb_context *usb_context ) noexcept
    : device_( std::move( device ) ), usb_context_( usb_context )
{
  // Default to the USB-advertised serial (Nano 300). Products that don't expose
  // it over USB overwrite this from factory data in setupCamera().
  serial_number_ = device_.serial;
}

SeekThermalCamera::~SeekThermalCamera()
{
  if ( usb_device_handle_ != nullptr ) {
    close();
  }
  if ( own_usb_context_ ) {
    libusb_exit( usb_context_ );
  }
}

void SeekThermalCamera::open()
{
  openDevice();
  try {
    // setupCamera() ends with SET_OPERATION_MODE=1, after which the camera
    // emits a deterministic boot sequence. tryConsumeStartupFrames() needs the
    // ft=8 startup shutter and the first ft=3 thermal frame to seed FFC / drift
    // state. If the device drops one of them we restart streaming (setupCamera
    // begins with SET_OPERATION_MODE=0) and retry.
    constexpr int kSetupAttempts = 5;
    for ( int attempt = 0; attempt < kSetupAttempts; ++attempt ) {
      setupCamera();
      if ( tryConsumeStartupFrames() )
        return;
      LOG_WARN( "Did not observe ft=8 + first ft=3 in startup frame budget on attempt "
                << ( attempt + 1 ) << "/" << kSetupAttempts << "; restarting camera." );
    }
    throw SeekSetupError( "Camera did not emit ft=8 + first ft=3 startup frames after " +
                          std::to_string( kSetupAttempts ) + " attempts" );
  } catch ( const std::exception & ) {
    // Clean up device before throwing.
    close();
    throw;
  }
}

void SeekThermalCamera::openDevice()
{
  int returncode;
  if ( usb_device_handle_ != nullptr ) {
    LOG_DEBUG( "Device already open!" );
    return;
  }
  if ( usb_context_ == nullptr ) {
    returncode = libusb_init( &usb_context_ );
    if ( returncode != 0 ) {
      throw USBError( "Failed to initialize libusb!", returncode );
    }
    own_usb_context_ = true;
  }

  // Find usb device
  libusb_device **devices;
  ssize_t device_count = libusb_get_device_list( usb_context_, &devices );
  if ( device_count < 0 ) {
    throw USBError( "Failed to get device list!", static_cast<int>( device_count ) );
  }
  libusb_device_descriptor desc = {};
  for ( ssize_t i = 0; i < device_count; ++i ) {
    libusb_device *device = devices[i];
    returncode = libusb_get_device_descriptor( device, &desc );
    if ( returncode != 0 ) {
      continue;
    }
    if ( desc.idVendor != device_._getVendorID() || desc.idProduct != device_._getProductID() ) {
      continue;
    }

    std::string port = bus_and_port_numbers_to_string( device );
    if ( device_.serial.empty() && device_.usb_port != port ) {
      // Port is only used if serial is not available
      continue;
    }

    returncode = libusb_open( device, &usb_device_handle_ );
    if ( returncode < 0 ) {
      int bus = libusb_get_bus_number( device );
      int device_address = libusb_get_device_address( device );
      LOG_WARN( "Failed to open device (" << std::setfill( '0' ) << std::setw( 3 ) << bus << ":"
                                          << std::setw( 3 ) << device_address
                                          << "): " << libusb_error_name( returncode ) );
      usb_device_handle_ = nullptr;
      continue;
    }
    std::string serial = get_usb_descriptor_ascii_string( usb_device_handle_, desc.iSerialNumber );
    if ( !device_.serial.empty() && serial != device_.serial ) {
      libusb_close( usb_device_handle_ );
      usb_device_handle_ = nullptr;
      continue;
    }
    // Either device has serial and serial matched, or no serial and port matched, we found it!
    break;
  }
  if ( usb_device_handle_ == nullptr ) {
    throw SeekSetupError( "Device not found!" );
  }

  int configuration_value;
  returncode = libusb_get_configuration( usb_device_handle_, &configuration_value );
  if ( returncode < 0 ) {
    throw USBError( "Failed to get configuration value!", returncode );
  }
  if ( configuration_value != 1 ) {
    returncode = libusb_set_configuration( usb_device_handle_, 1 );
    if ( returncode < 0 ) {
      throw USBError( "Failed to set configuration value!", returncode );
    }
  }
  returncode = libusb_claim_interface( usb_device_handle_, 0 );
  if ( returncode < 0 ) {
    throw USBError( "Failed to claim interface!", returncode );
  }
}

void SeekThermalCamera::close()
{
  if ( usb_device_handle_ == nullptr )
    return;
  // Best-effort: if the device is already in a bad state (e.g. close() is
  // called from open()'s catch block after a failed setup) the writes can
  // throw USBError. Swallow it so we still release/close the libusb handle.
  try {
    for ( int i = 0; i < 3; ++i ) {
      write( SeekDeviceCommand::SET_OPERATION_MODE, { 0x00, 0x00 } );
    }
  } catch ( const USBError &e ) {
    LOG_WARN( "USB error during close cleanup writes (ignored): " << e.what() );
  }
  libusb_release_interface( usb_device_handle_, 0 );
  libusb_close( usb_device_handle_ );
  usb_device_handle_ = nullptr;
  // Drop per-session state so the next open() picks up a fresh anchor.
  drift_anchor_set_ = false;
  drift_reference_anchor_ = 0.0;
  last_shutter_mean_ = 0.0;
  shutter_offset_.clear();
}

bool SeekThermalCamera::write( SeekDeviceCommand command, const std::vector<unsigned char> &data )
{
  if ( usb_device_handle_ == nullptr ) {
    throw USBError( "Device not open!", 0 );
  }

  int transferred = libusb_control_transfer(
      usb_device_handle_,
      LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
      static_cast<uint8_t>( command ), 0, 0, const_cast<unsigned char *>( data.data() ),
      data.size(), 1000 );
  if ( transferred < 0 ) {
    LOG_ERROR( "Failed to write command " << to_string( command ) << " ("
                                          << static_cast<int>( command )
                                          << "): " << libusb_error_name( transferred ) );
    return false;
  }
  if ( static_cast<size_t>( transferred ) != data.size() ) {
    LOG_ERROR( "Expected write command " << static_cast<int>( command ) << " to transfer "
                                         << data.size() << " bytes, but transferred "
                                         << transferred );
    return false;
  }
  LOG_DEBUG( "Wrote command " << to_string( command ) << " (" << static_cast<int>( command )
                              << ") data:\n"
                              << data_to_string( data ) );
  return true;
}

bool SeekThermalCamera::read( SeekDeviceCommand command, std::vector<unsigned char> &data )
{
  if ( usb_device_handle_ == nullptr ) {
    throw USBError( "Device not open!", 0 );
  }

  int transferred = libusb_control_transfer(
      usb_device_handle_, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
      static_cast<uint8_t>( command ), 0, 0, data.data(), data.size(), 1000 );
  if ( transferred < 0 ) {
    throw USBError( "Failed to read command " + to_string( command ) + ".", transferred );
  }
  if ( static_cast<size_t>( transferred ) != data.size() ) {
    LOG_ERROR( "Expected read command " << static_cast<int>( command ) << " to transfer "
                                        << data.size() << " bytes, but transferred " << transferred );
    return false;
  }
  LOG_DEBUG( "Read command " << to_string( command ) << " (" << static_cast<int>( command )
                             << ") data:\n"
                             << data_to_string( data ) );
  return true;
}

size_t SeekThermalCamera::getFrameSize() const { return getFrameWidth() * getFrameHeight() * 2; }

int SeekThermalCamera::getFrameWidth() const { return device_.getFrameWidth(); }

int SeekThermalCamera::getFrameHeight() const { return device_.getFrameHeight(); }

Framerate SeekThermalCamera::getMaxFramerate() const { return device_.getMaxFramerate(); }

GrabFrameResult SeekThermalCamera::_grabRawFrame( unsigned char **frame_data, size_t &size )
{
  std::lock_guard device_lock( device_mutex_ );
  if ( usb_device_handle_ == nullptr ) {
    return GrabFrameResult::DEVICE_NOT_OPEN;
  }
  int todo = device_._getFrameTransferTotalSize();
  if ( *frame_data == nullptr ) {
    size = todo;
    *frame_data = new unsigned char[size];
  }
  if ( size < static_cast<size_t>( todo ) ) {
    return GrabFrameResult::BUFFER_TOO_SMALL;
  }
  // Send transfer request
  {
    const u_int32_t device_total_size = htole32( device_._getFrameTransferDeviceRequestSize() );
    const auto *b = reinterpret_cast<const uint8_t *>( &device_total_size );
    if ( !write( SeekDeviceCommand::START_GET_IMAGE_TRANSFER, { b[0], b[1], b[2], b[3] } ) )
      return GrabFrameResult::FAILED_TO_START_TRANSFER;
  }
  GrabFrameResult result = GrabFrameResult::SUCCESS;
  const int request_size = device_._getFrameTransferRequestSize();
  int done = 0;
  unsigned char *buffer = *frame_data;
  while ( todo > 0 ) {
    int transferred;
    int error = libusb_bulk_transfer( usb_device_handle_, 0x81, buffer,
                                      std::min( request_size, todo ), &transferred, 1000 );
    if ( error != 0 ) {
      LOG_ERROR( "Failed to transfer frame data! Error: " << libusb_error_name( error ) );
      result = GrabFrameResult::TRANSFER_INCOMPLETE;
      break;
    }
    buffer += transferred;
    done += transferred;
    todo -= transferred;
    if ( todo != 0 && transferred == 0 ) {
      LOG_ERROR( "Frame transfer stopped prematurely! Received only "
                 << done << " out of " << device_._getFrameTransferTotalSize() << " bytes." );
      result = GrabFrameResult::TRANSFER_INCOMPLETE;
      break;
    }
  }
  return result;
}

bool SeekThermalCamera::triggerShutter()
{
  std::lock_guard device_lock( device_mutex_ );
  if ( usb_device_handle_ == nullptr ) {
    return false;
  }
  return write( SeekDeviceCommand::TOGGLE_SHUTTER, { 0xFC, 0x00, 0x04, 0x00 } );
}

GrabFrameResult SeekThermalCamera::grabRawCountsFrame( unsigned char **image_data, size_t &size,
                                                       FrameHeader *header )
{
  std::lock_guard device_lock( device_mutex_ );
  std::lock_guard buffer_lock( buffer_mutex_ );
  if ( usb_device_handle_ == nullptr ) {
    return GrabFrameResult::DEVICE_NOT_OPEN;
  }
  if ( image_data != nullptr && *image_data != nullptr && size < getFrameSize() ) {
    return GrabFrameResult::BUFFER_TOO_SMALL;
  }
  // Init buffer if not provided
  if ( image_data != nullptr && *image_data == nullptr ) {
    size = getFrameSize();
    *image_data = new unsigned char[size];
  }

  if ( buffer_.size() < static_cast<size_t>( device_._getFrameTransferTotalSize() ) ) {
    buffer_.resize( device_._getFrameTransferTotalSize() );
  }
  unsigned char *buffer = buffer_.data();
  size_t buffer_size = buffer_.size();
  if ( GrabFrameResult result = _grabRawFrame( &buffer, buffer_size );
       result != GrabFrameResult::SUCCESS ) {
    return result;
  }

  static hector_timeit::Timer timer( "FrameProcessing", hector_timeit::Timer::Default, false, true );
  hector_timeit::TimeBlock block( timer );
  const int header_size = device_._getFrameHeaderSize();
  const size_t pixel_count =
      static_cast<size_t>( getFrameWidth() ) * static_cast<size_t>( getFrameHeight() );
  FrameHeader internal_header(
      device_.type,
      std::vector<unsigned char>(
          buffer_.begin(),
          buffer_.begin() +
              std::min<size_t>( buffer_size, FrameHeader::GetMinHeaderSize( device_.type ) ) ) );
  const FrameType frame_type = internal_header.getFrameType();
  if ( header != nullptr ) {
    *header = internal_header;
  }

  // Periodic shutter cycle: update host-side FFC reference. Mean-preserving —
  // the shutter blade presents a roughly uniform thermal source, so per-pixel
  // variation in ft=1 is fixed-pattern offset stored as `mean - ft1[i]` and
  // added to each scene pixel; the frame mean (and thus the calibration
  // anchor) is preserved.
  if ( frame_type == FrameType::CALIBRATION_FRAME ) {
    LOG_DEBUG( "Shutter (ft=1) frame received, refreshing FFC reference" );
    std::vector<uint16_t> shutter( pixel_count );
    extractFrame( buffer_.data() + header_size, reinterpret_cast<unsigned char *>( shutter.data() ) );
    applyShutterReference( shutter.data(), pixel_count );
    if ( c0_source_ == C0Source::CameraAuto )
      updateTemperatureCalibration( last_shutter_mean_,
                                    computePadDriftSignal( buffer_.data(), buffer_size ) );
  }

  if ( image_data == nullptr )
    return GrabFrameResult::SUCCESS;

  extractFrame( buffer_.data() + header_size, *image_data );
  if ( frame_type != FrameType::THERMAL_FRAME )
    return GrabFrameResult::SUCCESS;

  // Thermal-only count pipeline: shutter → dead-pixel → vignette → drift.
  auto *pixels = reinterpret_cast<uint16_t *>( *image_data );
  if ( shutter_correction_enabled_ && shutter_offset_.size() == pixel_count ) {
    for ( size_t i = 0; i < pixel_count; ++i ) {
      const int32_t corrected = static_cast<int32_t>( pixels[i] ) + shutter_offset_[i];
      pixels[i] = static_cast<uint16_t>( std::clamp( corrected, 0, 0xFFFF ) );
    }
  }
  if ( calibration_.dead_pixels ) {
    calibration_.dead_pixels->apply( pixels );
  }
  if ( calibration_.vignette ) {
    calibration_.vignette->apply( pixels );
  }
  // In-band substrate-drift compensation; see setDriftCompensationEnabled()
  // docstring for the full model.
  int32_t drift_offset_int = 0;
  const bool drift_active = substrate_drift_coefficient_ > 0.0 && drift_compensation_enabled_;
  if ( drift_active && drift_anchor_set_ ) {
    const double pad_drift_signal = computePadDriftSignal( buffer_.data(), buffer_size );
    if ( pad_drift_signal > 0.0 ) {
      const double pad_term =
          substrate_drift_coefficient_ * ( pad_drift_signal - drift_reference_anchor_ );
      drift_offset_int = static_cast<int32_t>( std::lround( pad_term ) );
    }
  }
  if ( drift_offset_int != 0 ) {
    for ( size_t i = 0; i < pixel_count; ++i ) {
      const int32_t corrected = static_cast<int32_t>( pixels[i] ) - drift_offset_int;
      pixels[i] = static_cast<uint16_t>( std::clamp( corrected, 0, 0xFFFF ) );
    }
  }
  return GrabFrameResult::SUCCESS;
}

GrabFrameResult SeekThermalCamera::grabFrame( unsigned char **image_data, size_t &size,
                                              FrameHeader *header )
{
  FrameHeader internal_header;
  if ( GrabFrameResult result = grabRawCountsFrame( image_data, size, &internal_header );
       result != GrabFrameResult::SUCCESS )
    return result;
  if ( header != nullptr )
    *header = internal_header;
  if ( image_data == nullptr || internal_header.getFrameType() != FrameType::THERMAL_FRAME )
    return GrabFrameResult::SUCCESS;

  // Map drift-compensated counts to centi-Kelvin.
  std::lock_guard buffer_lock( buffer_mutex_ );
  const size_t pixel_count =
      static_cast<size_t>( getFrameWidth() ) * static_cast<size_t>( getFrameHeight() );
  auto *pixels = reinterpret_cast<uint16_t *>( *image_data );
  const TemperatureCalibration &cal_to_apply = *calibration_.temperature;
  for ( size_t i = 0; i < pixel_count; ++i ) { pixels[i] = cal_to_apply.apply( pixels[i] ); }
  return GrabFrameResult::SUCCESS;
}

double SeekThermalCamera::computePadDriftSignal( const unsigned char *transfer_buffer,
                                                 size_t transfer_buffer_size ) const
{
  const int header_size = device_._getFrameHeaderSize();
  const int row_u16 = device_._getRowStep() / 2;
  const int visible_rows = getFrameHeight();
  const int pad_col = getFrameWidth();
  if ( transfer_buffer_size <
       static_cast<size_t>( header_size ) +
           static_cast<size_t>( visible_rows ) * static_cast<size_t>( row_u16 ) * 2 ) {
    return 0.0;
  }
  const auto *body_u16 = reinterpret_cast<const uint16_t *>( transfer_buffer + header_size );
  uint64_t sum = 0;
  int count = 0;
  for ( int y = 0; y < visible_rows; ++y ) {
    // Skip sentinels (0 / 0xFFFF). They appear in some startup/test transfers
    // and would skew the mean.
    const uint16_t v = le16toh( body_u16[y * row_u16 + pad_col] );
    if ( v == 0 || v == 0xFFFF )
      continue;
    sum += v;
    ++count;
  }
  return count > 0 ? static_cast<double>( sum ) / count : 0.0;
}

void SeekThermalCamera::applyShutterReference( const uint16_t *shutter, size_t pixel_count )
{
  // Dead-pixel sentinels (0 / 0xFFFF) are excluded from the mean — otherwise a
  // single 0xFFFF pixel skews it by ~tens of counts and biases every per-pixel
  // offset, blowing up the absolute calibration by tens of °C.
  uint64_t sum = 0;
  size_t count = 0;
  for ( size_t i = 0; i < pixel_count; ++i ) {
    const uint16_t v = shutter[i];
    if ( v == 0 || v == 0xFFFF )
      continue;
    sum += v;
    ++count;
  }
  if ( count == 0 ) {
    LOG_WARN( "Shutter reference frame had no valid pixels; keeping previous "
              "shutter_offset_." );
    return;
  }
  const int32_t mean = static_cast<int32_t>( sum / count );
  last_shutter_mean_ = static_cast<double>( mean );
  if ( shutter_offset_.size() != pixel_count )
    shutter_offset_.assign( pixel_count, 0 );
  for ( size_t i = 0; i < pixel_count; ++i ) {
    const int32_t v = shutter[i];
    shutter_offset_[i] = ( v == 0 || v == 0xFFFF ) ? 0 : mean - v;
  }
}

bool SeekThermalCamera::tryConsumeStartupFrames()
{
  // Boot order is `4, 9, 14, 25, 26, 27, 28, 8, 7` then the first normal
  // shutter cycle (ft=6/1/20) and the first ft=3 thermal (see FRAME-TYPES.md).
  // The budget covers the boot tail plus that shutter cycle and first thermal so
  // a stray transfer error doesn't immediately force a setupCamera() retry.
  constexpr int kBudget = 24;
  const size_t pixel_count =
      static_cast<size_t>( getFrameWidth() ) * static_cast<size_t>( getFrameHeight() );
  const int header_size = device_._getFrameHeaderSize();
  std::vector<unsigned char> transfer( device_._getFrameTransferTotalSize() );
  std::vector<uint16_t> extracted( pixel_count );
  bool ft8_seen = false;
  double boot_shutter_pad = 0.0;
  for ( int i = 0; i < kBudget; ++i ) {
    unsigned char *buf = transfer.data();
    size_t buf_size = transfer.size();
    if ( _grabRawFrame( &buf, buf_size ) != GrabFrameResult::SUCCESS )
      return false;
    FrameHeader header(
        device_.type,
        std::vector<unsigned char>(
            transfer.begin(),
            transfer.begin() +
                std::min<size_t>( buf_size, FrameHeader::GetMinHeaderSize( device_.type ) ) ) );
    const FrameType ft = header.getFrameType();
    // Seed the per-unit firmware SLOPE c1 = 100 / counts (row 1 byte 16, present
    // in every transfer) unless one is already installed. The absolute offset c0
    // is filled in once the boot shutter (ft=8) and drift anchor (first ft=3) are
    // both known (below); until then it stays 0.
    if ( !calibration_.temperature ) {
      const uint16_t counts = header.getCountsPer100Celsius();
      if ( counts != 0 ) {
        TemperatureCalibration t;
        t.c1 = 100.0 / static_cast<double>( counts );
        t.c0 = 0.0;
        calibration_.temperature = t;
      }
    }
    if ( !ft8_seen && ft == FrameType::STARTUP_CALIBRATION_FRAME ) {
      extractFrame( transfer.data() + header_size,
                    reinterpret_cast<unsigned char *>( extracted.data() ) );
      applyShutterReference( extracted.data(), pixel_count );
      ft8_seen = shutter_offset_.size() == pixel_count;
      if ( ft8_seen ) {
        boot_shutter_pad = computePadDriftSignal( transfer.data(), buf_size );
        LOG_DEBUG( "[ffc] startup shutter reference captured (mean="
                   << last_shutter_mean_ << " pad=" << boot_shutter_pad << ")" );
      }
    } else if ( !drift_anchor_set_ && ft == FrameType::THERMAL_FRAME ) {
      // Anchor in-band drift compensation to the FIRST real thermal pad column,
      // which reflects this boot's live substrate state (not the stored ft=4
      // shading map, whose pad does not track the boot substrate).
      const double pad = computePadDriftSignal( transfer.data(), buf_size );
      if ( pad > 0.0 ) {
        drift_reference_anchor_ = pad;
        drift_anchor_set_ = true;
        LOG_DEBUG( "[drift] startup reference set from first THERMAL_FRAME: pad="
                   << drift_reference_anchor_ );
        // Seed the camera-only absolute offset c0 from the boot shutter blade as
        // a blackbody at the factory reference temperature, drift-normalized to
        // this boot's substrate state:
        //   c0 = factory_T_ref_ - c1*(shutter_mean - K*(shutter_pad - pad_ref))
        // The factory characterizes the unit at T_ref (factory page 0x20); the
        // drift comp removes the boot substrate state, so the live housing NTC --
        // unreliable when the camera boots warm -- is not needed. Re-anchored at
        // every later shutter in grabRawCountsFrame.
        // Skipped when a host temperature calibration is provided.
        if ( factory_T_ref_ > 0.0 && calibration_.temperature && c0_source_ != C0Source::Host &&
             substrate_drift_coefficient_ > 0.0 && boot_shutter_pad > 0.0 ) {
          c0_source_ = C0Source::CameraAuto;
          updateTemperatureCalibration( last_shutter_mean_, boot_shutter_pad );
          LOG_DEBUG( "[c0] camera-only offset c0=" << calibration_.temperature->c0
                                                   << " (T_ref=" << factory_T_ref_ << ")" );
        }
      }
    }
    if ( ft8_seen && drift_anchor_set_ ) {
      if ( !calibration_.temperature ) {
        // If no slope could be found, install identity (cK == raw counts).
        calibration_.temperature = TemperatureCalibration{ -273.15, 0.01 };
      }
      return true;
    }
  }
  return false;
}

void SeekThermalCamera::extractFrame( const unsigned char *__restrict__ data,
                                      unsigned char *__restrict__ frame_data )
{
  const int width = device_.getFrameWidth();
  const int height = device_.getFrameHeight();
  const int row_step = device_._getRowStep() / 2;
  auto *data_in = reinterpret_cast<const uint16_t *>( data );
  auto *data_out = reinterpret_cast<uint16_t *>( frame_data );
  // Pass good pixels through unchanged; fall back to a 3x3 gaussian over the
  // valid neighbours only when the center pixel itself is dead/saturated.
  // Reads convert from on-wire LE to host once here; all downstream
  // processing operates on host-endian pixels.
  const int filter_weights[9] = { 1, 2, 1, 2, 4, 2, 1, 2, 1 };
  for ( int y = 0; y < height; ++y ) {
    for ( int x = 0; x < width; ++x ) {
      const int out_index = y * width + x;
      const uint16_t center_value = le16toh( data_in[y * row_step + x] );
      if ( center_value != 0 && center_value != 0xffff ) {
        data_out[out_index] = center_value;
        continue;
      }
      int sum = 0;
      int count = 0;
      for ( int k = -1; k <= 1; ++k ) {
        for ( int m = -1; m <= 1; ++m ) {
          if ( ( k == 0 && m == 0 ) || y + k < 0 || y + k >= height || x + m < 0 || x + m >= width ) {
            continue;
          }
          const int index = ( y + k ) * row_step + x + m;
          const int value = le16toh( data_in[index] );
          if ( value == 0 || value == 0xffff )
            continue;
          sum += value * filter_weights[( k + 1 ) * 3 + m + 1];
          count += filter_weights[( k + 1 ) * 3 + m + 1];
        }
      }
      data_out[out_index] = count == 0 ? 0 : static_cast<uint16_t>( sum / count );
    }
  }
}

void SeekThermalCamera::setShutterCorrectionEnabled( bool enabled )
{
  std::lock_guard buffer_lock( buffer_mutex_ );
  shutter_correction_enabled_ = enabled;
}

void SeekThermalCamera::setDriftCompensationEnabled( bool enabled )
{
  std::lock_guard buffer_lock( buffer_mutex_ );
  drift_compensation_enabled_ = enabled;
}

void SeekThermalCamera::setCalibration( CameraCalibration cal )
{
  if ( cal.vignette &&
       ( cal.vignette->width != getFrameWidth() || cal.vignette->height != getFrameHeight() ) ) {
    throw std::invalid_argument( "Vignette correction dimensions do not match camera frame" );
  }
  if ( cal.dead_pixels && ( cal.dead_pixels->width() != getFrameWidth() ||
                            cal.dead_pixels->height() != getFrameHeight() ) ) {
    throw std::invalid_argument( "Dead-pixel mask dimensions do not match camera frame" );
  }
  std::lock_guard buffer_lock( buffer_mutex_ );
  // Keep the active temperature mapping (factory default from open(), or a
  // previously installed one) when the incoming calibration omits it. A host
  // temperature section takes ownership of c0 and stops the per-shutter
  // camera-only re-anchoring.
  if ( cal.temperature ) {
    // Host owns c0/c1 so a later open()/reopen() does not seed the camera-only
    // c0 over it (the calibration may be installed before open()).
    c0_source_ = C0Source::Host;
    // Pin the in-band drift anchor to the reference this c0 was fit against so
    // the fixed offset stays valid across reboots; otherwise the per-boot
    // first-thermal anchor (set in tryConsumeStartupFrames) would shift it by
    // c1*K*(boot_pad - fit_pad). Unset pad_ref keeps that per-boot anchor.
    if ( !std::isnan( cal.temperature->pad_ref ) ) {
      drift_reference_anchor_ = cal.temperature->pad_ref;
      drift_anchor_set_ = true;
    }
  } else {
    cal.temperature = calibration_.temperature;
  }
  calibration_ = std::move( cal );
}

void SeekThermalCamera::updateTemperatureCalibration( double shutter_mean, double shutter_pad )
{
  // c0 = factory_T_ref_ - c1 * (shutter_mean drift-normalized to the boot pad
  // reference). The shutter blade is a blackbody at the factory reference
  // temperature once the boot substrate state is removed. See
  // tryConsumeStartupFrames for the rationale.
  if ( factory_T_ref_ <= 0.0 || !calibration_.temperature || substrate_drift_coefficient_ <= 0.0 ||
       !drift_anchor_set_ || shutter_pad <= 0.0 )
    return;
  const double dc_shutter =
      shutter_mean - substrate_drift_coefficient_ * ( shutter_pad - drift_reference_anchor_ );
  calibration_.temperature->c0 = factory_T_ref_ - calibration_.temperature->c1 * dc_shutter;
}

std::string SeekThermalCamera::readChipID()
{
  std::vector<unsigned char> data( 12 );
  read( SeekDeviceCommand::READ_CHIP_ID, data );
  std::stringstream ss;
  for ( size_t i = 0; i < data.size(); ++i ) {
    ss << std::hex << std::setfill( '0' ) << std::setw( 2 ) << static_cast<int>( data[i] );
  }
  return ss.str();
}

std::string SeekThermalCamera::readFirmwareInfo()
{
  std::vector<unsigned char> data( 4 );
  read( SeekDeviceCommand::GET_FIRMWARE_INFO, data );
  return std::string( data.begin(), data.end() );
}

std::string to_string( GrabFrameResult result )
{
  switch ( result ) {
  case GrabFrameResult::SUCCESS:
    return "SUCCESS";
  case GrabFrameResult::DEVICE_NOT_OPEN:
    return "DEVICE_NOT_OPEN";
  case GrabFrameResult::FAILED_TO_START_TRANSFER:
    return "FAILED_TO_START_TRANSFER";
  case GrabFrameResult::TRANSFER_INCOMPLETE:
    return "TRANSFER_INCOMPLETE";
  case GrabFrameResult::BUFFER_TOO_SMALL:
    return "BUFFER_TOO_SMALL";
  case GrabFrameResult::UNKNOWN_ERROR:
    return "UNKNOWN_ERROR";
  }
  return "INVALID";
}
} // namespace openseekthermal
