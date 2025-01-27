// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "openseekthermal/detail//cameras/seek_thermal_compact_pro.hpp"
#include "openseekthermal/detail/exceptions.hpp"
#include <cassert>

#include "../logging.hpp"

namespace openseekthermal
{

SeekThermalCompactPro::SeekThermalCompactPro( const openseekthermal::SeekDevice &device,
                                              libusb_context *usb_context )
    : SeekThermalCamera( device, usb_context )
{
  assert( device.type == SeekDevice::Type::SeekThermalCompactPro );
}

SeekThermalCompactPro::~SeekThermalCompactPro() = default;

void SeekThermalCompactPro::setupCamera()
{
  if ( !write( SeekDeviceCommand::TARGET_PLATFORM, { 0x01 } ) ) {
    close();
    openDevice();
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
      throw SeekSetupError( "Failed to set operation mode to off! Mode is: " +
                            data_to_string( data ) );
    if ( !read( SeekDeviceCommand::GET_OPERATION_MODE, data ) )
      throw SeekSetupError( "Failed to read operation mode!" );

  } while ( data[0] != 0x00 && data[1] != 0x00 );

  if ( !write( SeekDeviceCommand::SET_FACTORY_SETTINGS_FEATURES,
               { 0x06, 0x00, 0x08, 0x00, 0x00, 0x00 } ) )
    throw SeekSetupError( "Failed to set factory settings features!" );
  if ( !write( SeekDeviceCommand::SET_FIRMWARE_INFO_FEATURES, { 0x17, 0x00 } ) )
    throw SeekSetupError( "Failed to set firmware info features to 0x17 0x00!" );
  if ( !write( SeekDeviceCommand::SET_FACTORY_SETTINGS_FEATURES,
               { 0x01, 0x00, 0x00, 0x06, 0x00, 0x00 } ) )
    throw SeekSetupError( "Failed to set factory settings features!" );
  if ( !write( SeekDeviceCommand::SET_FACTORY_SETTINGS_FEATURES,
               { 0x01, 0x00, 0x01, 0x06, 0x00, 0x00 } ) )
    throw SeekSetupError( "Failed to set factory settings features!" );

  for ( int addr = 0; addr < 0xA00; addr += 0x20 ) {
    uint16_t addr_le = htole16( addr );
    auto *addr_bytes = reinterpret_cast<const unsigned char *>( &addr_le );
    if ( !write( SeekDeviceCommand::SET_FACTORY_SETTINGS_FEATURES,
                 { 0x20, 0x00, addr_bytes[0], addr_bytes[1], 0x00, 0x00 } ) )
      throw SeekSetupError(
          "Failed to set factory settings features to 0x20 0x00 0x00 0x00 0x00 0x00!" );
    if ( data.resize( 64 ); !read( SeekDeviceCommand::GET_FACTORY_SETTINGS, data ) )
      throw SeekSetupError( "Failed to read factory settings features!" );
  }

  if ( !write( SeekDeviceCommand::SET_FIRMWARE_INFO_FEATURES, { 0x15, 0x00 } ) )
    throw SeekSetupError( "Failed to set firmware info features to 0x15 0x00!" );

  if ( data.resize( 4 ); !read( SeekDeviceCommand::GET_ERROR_CODE, data ) )
    throw SeekSetupError( "Failed to read error code!" );
  if ( data[0] != 0x00 || data[1] != 0x00 || data[2] != 0x00 || data[3] != 0x00 )
    throw SeekSetupError( "Camera reported error code during setup: " + data_to_string( data ) );

  if ( !write( SeekDeviceCommand::SET_IMAGE_PROCESSING_MODE, { 0x08, 0x00 } ) )
    throw SeekSetupError( "Failed to set image processing mode!" );
  retries = 0;
  if ( data.resize( 2 ); !read( SeekDeviceCommand::GET_OPERATION_MODE, data ) )
    throw SeekSetupError( "Failed to read operation mode!" );
  do {
    if ( ++retries > 10 )
      throw SeekSetupError( "Failed to set operation mode to on after 10 attempts!" );
    uint16_t operation_mode = le16toh( *reinterpret_cast<uint16_t *>( data.data() ) );
    if ( !write( SeekDeviceCommand::SET_OPERATION_MODE, { 0x01, 0x00 } ) ) {
      if ( data.resize( 4 ); !read( SeekDeviceCommand::GET_ERROR_CODE, data ) && data[0] == 0x00 &&
                             data[1] == 0x00 && data[2] == 0x00 && data[3] == 0x00 )
        throw SeekSetupError( "Failed to set operation mode to on from " +
                              std::to_string( operation_mode ) + "!" );
      throw SeekSetupError( "Failed to set operation mode to on from " +
                            std::to_string( operation_mode ) + "! Error: " + data_to_string( data ) );
    }
    if ( data.resize( 2 ); !read( SeekDeviceCommand::GET_OPERATION_MODE, data ) )
      throw SeekSetupError( "Failed to read operation mode!" );
  } while ( data[0] != 0x01 && data[1] != 0x00 );
}
} // namespace openseekthermal
