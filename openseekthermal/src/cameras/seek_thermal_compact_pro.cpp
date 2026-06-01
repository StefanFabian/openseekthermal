// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "openseekthermal/detail//cameras/seek_thermal_compact_pro.hpp"
#include "openseekthermal/detail/exceptions.hpp"
#include <cassert>
#include <cstring>

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
    // Pages are 64 bytes on a 32-byte stride, but the overlapping first 32
    // bytes of each non-initial page are inconsistent with the second half of
    // the previous page (the camera returns different data for the same
    // factory address depending on which page request was used). The
    // factory anchor table (5 records × 4 f32) starts at factory byte 0x44:
    //   anchor[0] = (raw_at_T_ref, ?, T_ref_C=22, count=3)
    //   anchor[1] = (BB_T_C, BB_lnR, anchor1_f2, BB_emissivity_x100)
    //   anchor[2] = (lnR_ref, 4·B_ntc, T_factory_K, ?)
    // The housing-NTC Beta thermometer needs anchor[1].f[2] and anchor[2].f[1].
    if ( addr == 0x20 && data.size() >= 0x40 ) {
      float v;
      std::memcpy( &v, data.data() + 0x24, sizeof( v ) );
      factory_raw_at_T_ref_ = v;
      std::memcpy( &v, data.data() + 0x2c, sizeof( v ) );
      factory_T_ref_ = v;
      float anchor1_f2;
      std::memcpy( &anchor1_f2, data.data() + 0x3c, sizeof( anchor1_f2 ) );
      if ( factory_T_ref_ > 0.0f )
        factory_housing_K_ = factory_raw_at_T_ref_ - anchor1_f2 / factory_T_ref_;
    }
    // Experimentally identified Beta-NTC coefficient B = factory float32 at
    // factory byte 0x58 (page 0x40 rel-offset 0x18). Nano 3812 K / CP 3905 K
    if ( addr == 0x40 && data.size() >= 0x1c ) {
      float beta;
      std::memcpy( &beta, data.data() + 0x18, sizeof( beta ) );
      factory_housing_B_ = beta;
    }
  }
  if ( factory_raw_at_T_ref_ > 0.0 && factory_housing_K_ > 0.0 && factory_housing_B_ > 0.0 &&
       factory_housing_K_ < factory_raw_at_T_ref_ ) {
    factory_housing_valid_ = true;
  }
  // Substrate-drift slope, fit on `contwarmup-compactpro-80C-20260517-171127`
  // (R²=1.000, slope 1.04). The CP shows tighter pad-vs-scene coupling than
  // the Nano (likely a substrate/optics-geometry effect).
  substrate_drift_coefficient_ = 1.04;

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
