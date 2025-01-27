// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "openseekthermal/detail/cameras/seek_thermal_camera.hpp"
#include "openseekthermal/detail/exceptions.hpp"
#include <libusb-1.0/libusb.h>

#include <utility>

#include "../helpers.hpp"
#include "../logging.hpp"
#include "../timer.hpp"

namespace openseekthermal
{

SeekThermalCamera::SeekThermalCamera( SeekDevice device, libusb_context *usb_context ) noexcept
    : device_( std::move( device ) ), usb_context_( usb_context )
{
  calibration_frame_.resize( getFrameWidth() * getFrameHeight(), offset_ );
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
    setupCamera();
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
    int returncode = libusb_init( &usb_context_ );
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
  for ( int i = 0; i < 3; ++i ) { write( SeekDeviceCommand::SET_OPERATION_MODE, { 0x00, 0x00 } ); }
  libusb_release_interface( usb_device_handle_, 0 );
  libusb_close( usb_device_handle_ );
  usb_device_handle_ = nullptr;
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
    int error =
        libusb_bulk_transfer( usb_device_handle_, 0x81, buffer, request_size, &transferred, 1000 );
    if ( error != 0 ) {
      LOG_ERROR( "Failed to transfer frame data! Error: " << libusb_error_name( error ) );
      result = GrabFrameResult::TRANSFER_INCOMPLETE;
      break;
    }
    buffer += transferred;
    done += transferred;
    todo -= transferred;
    if ( todo != 0 && request_size != transferred ) {
      LOG_ERROR( "Frame transfer stopped prematurely! Received only "
                 << done << " out of " << device_._getFrameTransferTotalSize() << " bytes." );
      result = GrabFrameResult::TRANSFER_INCOMPLETE;
      break;
    }
  }
  return result;
}

GrabFrameResult SeekThermalCamera::grabFrame( unsigned char **image_data, size_t &size,
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

  // Get actual frame image_data
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
  if ( device_._isCalibrationFrame( buffer_ ) ) {
    LOG_DEBUG( "Calibration frame detected, skipping" );
    extractFrame( buffer_.data() + header_size,
                  reinterpret_cast<unsigned char *>( calibration_frame_.data() ) );
  }

  if ( header != nullptr ) {
    header->type_ = device_.type;
    if ( header->data_.size() < FrameHeader::GetMinHeaderSize( device_.type ) ) {
      header->data_.resize( header->GetMinHeaderSize( device_.type ) );
    }
    std::copy( buffer_.begin(), buffer_.begin() + std::min<int>( buffer_size, header->data_.size() ),
               header->data_.begin() );
  }
  if ( image_data != nullptr ) {
    extractFrame( buffer_.data() + header_size, *image_data );
    applyCalibration( *image_data );
  }
  return GrabFrameResult::SUCCESS;
}

void SeekThermalCamera::extractFrame( const unsigned char *__restrict__ data,
                                      unsigned char *__restrict__ frame_data )
{
  const int width = device_.getFrameWidth();
  const int height = device_.getFrameHeight();
  const int row_step = device_._getRowStep() / 2;
  auto *data_in = reinterpret_cast<const uint16_t *>( data );
  auto *data_out = reinterpret_cast<uint16_t *>( frame_data );
  // Apply a gaussian filter using only the set pixels
  int filter_weights[9] = { 1, 2, 1, 2, 4, 2, 1, 2, 1 };
  for ( int y = 0; y < height; ++y ) {
    for ( int x = 0; x < width; ++x ) {
      const int out_index = y * width + x;
      int sum = 0;
      int count = 0;
      for ( int k = -1; k <= 1; ++k ) {
        for ( int m = -1; m <= 1; ++m ) {
          if ( y + k < 0 || y + k >= height || x + m < 0 || x + m >= width ) {
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
      data_out[out_index] = sum == 0 ? 0 : htole16( sum / count );
    }
  }
}

void SeekThermalCamera::applyCalibration( unsigned char *frame_data )
{
  auto *data = reinterpret_cast<uint16_t *>( frame_data );
  for ( size_t i = 0; i < calibration_frame_.size(); ++i ) {
    data[i] = htole16( le16toh( data[i] ) + offset_ - le16toh( calibration_frame_[i] ) );
  }
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
