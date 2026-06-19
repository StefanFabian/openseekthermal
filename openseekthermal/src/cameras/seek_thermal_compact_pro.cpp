// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "openseekthermal/detail//cameras/seek_thermal_compact_pro.hpp"
#include "openseekthermal/detail/exceptions.hpp"
#include <algorithm>
#include <cassert>
#include <cctype>
#include <cstring>
#include <string>

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

  } while ( data[0] != 0x00 || data[1] != 0x00 );

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
    // The Compact Pro does not advertise a serial over USB. Factory page 0x00
    // carries the 12-char ASCII unit serial at relative offset 0x10
    // (null-padded), mirroring the Nano's USB serial format.
    if ( addr == 0x00 && data.size() >= 0x10 ) {
      std::string s( reinterpret_cast<const char *>( data.data() ) + 0x10,
                     std::min<size_t>( 12, data.size() - 0x10 ) );
      s.erase( std::find_if( s.begin(), s.end(),
                             []( unsigned char c ) { return c == '\0' || std::isprint( c ) == 0; } ),
               s.end() );
      if ( !s.empty() )
        serial_number_ = s;
    }
    // Factory page 0x20 holds the unit's reference temperature as a
    // little-endian float32 at relative offset 0x2c (~22 °C on most units). It
    // is the absolute-temperature anchor; the calibration frame is treated as a
    // blackbody at this temperature.
    if ( addr == 0x20 && data.size() >= 0x30 ) {
      float v;
      std::memcpy( &v, data.data() + 0x2c, sizeof( v ) );
      factory_T_ref_ = v;
    }
  }
  // Per-product substrate-drift slope: scene-pixel raw counts per pad-column
  // count for the in-band drift compensation. See setDriftCompensationEnabled().
  substrate_drift_coefficient_ = 1.17;

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
  } while ( data[0] != 0x01 || data[1] != 0x00 );
}
} // namespace openseekthermal
