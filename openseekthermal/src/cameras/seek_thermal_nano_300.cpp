// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "openseekthermal/detail//cameras/seek_thermal_nano_300.hpp"
#include "openseekthermal/detail/exceptions.hpp"
#include <cassert>
#include <cstring>
#include <iomanip>
#include <sstream>

#include "../logging.hpp"

namespace openseekthermal
{

SeekThermalNano300::SeekThermalNano300( const openseekthermal::SeekDevice &device,
                                        libusb_context *usb_context )
    : SeekThermalCamera( device, usb_context )
{
  assert( device.type == SeekDevice::Type::SeekThermalNano300 );
}

SeekThermalNano300::~SeekThermalNano300() = default;

void SeekThermalNano300::setupCamera()
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
    if ( !read( SeekDeviceCommand::GET_OPERATION_MODE, data ) )
      throw SeekSetupError( "Failed to read chip ID!" );

  } while ( data[0] != 0x00 || data[1] != 0x00 );

  if ( !write( SeekDeviceCommand::SET_IMAGE_PROCESSING_MODE, { 0x08, 0x00 } ) )
    throw SeekSetupError( "Failed to set image processing mode!" );
  if ( !write( SeekDeviceCommand::SET_FACTORY_SETTINGS_FEATURES,
               { 0x08, 0x00, 0x02, 0x06, 0x00, 0x00 } ) )
    throw SeekSetupError( "Failed to set factory settings features!" );
  if ( data.resize( 16 ); !read( SeekDeviceCommand::GET_FACTORY_SETTINGS, data ) )
    throw SeekSetupError( "Failed to read factory settings features!" );
  if ( data.resize( 4 ); !read( SeekDeviceCommand::GET_FIRMWARE_INFO, data ) )
    throw SeekSetupError( "Failed to read firmware info!" );
  if ( !write( SeekDeviceCommand::SET_FACTORY_SETTINGS_FEATURES,
               { 0x08, 0x00, 0x02, 0x06, 0x00, 0x00 } ) )
    throw SeekSetupError( "Failed to set factory settings features!" );
  if ( data.resize( 16 ); !read( SeekDeviceCommand::GET_FACTORY_SETTINGS, data ) )
    throw SeekSetupError( "Failed to read factory settings features!" );
  if ( !write( SeekDeviceCommand::SET_FIRMWARE_INFO_FEATURES, { 0x17, 0x00 } ) )
    throw SeekSetupError( "Failed to set firmware info features to 0x17 0x00!" );
  if ( data.resize( 64 ); !read( SeekDeviceCommand::GET_FIRMWARE_INFO, data ) )
    throw SeekSetupError( "Failed to read firmware info!" );
  if ( data.resize( 4 ); !read( SeekDeviceCommand::GET_FIRMWARE_INFO, data ) )
    throw SeekSetupError( "Failed to read firmware info!" );
  if ( data.resize( 12 ); !read( SeekDeviceCommand::READ_CHIP_ID, data ) )
    throw SeekSetupError( "Failed to read chip ID!" );
  if ( !write( SeekDeviceCommand::SET_FIRMWARE_INFO_FEATURES, { 0x15, 0x00 } ) )
    throw SeekSetupError( "Failed to set firmware info features to 0x15 0x00!" );
  if ( data.resize( 64 ); !read( SeekDeviceCommand::GET_FIRMWARE_INFO, data ) )
    throw SeekSetupError( "Failed to read firmware info!" );
  if ( data.resize( 12 ); !read( SeekDeviceCommand::READ_CHIP_ID, data ) )
    throw SeekSetupError( "Failed to read chip ID!" );
  if ( data.resize( 4 ); !read( SeekDeviceCommand::GET_FIRMWARE_INFO, data ) )
    throw SeekSetupError( "Failed to read firmware info!" );
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
      // anchor[1].f[2] = factory byte 0x5c → rel-offset 0x3c in this page.
      float anchor1_f2;
      std::memcpy( &anchor1_f2, data.data() + 0x3c, sizeof( anchor1_f2 ) );
      if ( factory_T_ref_ > 0.0f )
        factory_housing_K_ = factory_raw_at_T_ref_ - anchor1_f2 / factory_T_ref_;
    }
    // anchor[2] straddles the page boundary. Its f[1] (= 4·B_NTC) lives in the
    // first half of the page at addr 0x40 at rel-offset 0x08 (= factory byte
    // 0x48 logically, which is anchor[2].f[1] under the 32-byte-shift page
    // overlap convention — see the comment above).
    if ( addr == 0x40 && data.size() >= 0x0c ) {
      float anchor2_f1;
      std::memcpy( &anchor2_f1, data.data() + 0x08, sizeof( anchor2_f1 ) );
      factory_housing_B_ = anchor2_f1 / 4.0f;
    }
  }
  if ( factory_raw_at_T_ref_ > 0.0 && factory_housing_K_ > 0.0 && factory_housing_B_ > 0.0 &&
       factory_housing_K_ < factory_raw_at_T_ref_ ) {
    factory_housing_valid_ = true;
  }
  // Substrate-drift slope: scene-pixel raw counts per pad-column count for
  // the in-band drift compensation. Empirically fit across Nano A streaming
  // warmup (`livedrift_streamfit_20260526.csv`) and 60/80 °C cont-warmup
  // captures — slope clusters at 1.15–1.27, K=1.15 keeps the residual
  // envelope below 1.5 °C across the 15-min warmup at static scene.
  substrate_drift_coefficient_ = 1.15;
  LOG_DEBUG( "[housing-ntc] H_ref=" << factory_raw_at_T_ref_ << " T_ref=" << factory_T_ref_
                                    << " K=" << factory_housing_K_ << " B=" << factory_housing_B_
                                    << " valid=" << factory_housing_valid_ );
  if ( !write( SeekDeviceCommand::SET_FIRMWARE_INFO_FEATURES, { 0x15, 0x00 } ) )
    throw SeekSetupError( "Failed to set firmware info features to 0x15 0x00!" );
  if ( data.resize( 64 ); !read( SeekDeviceCommand::GET_FIRMWARE_INFO, data ) )
    throw SeekSetupError( "Failed to read firmware info!" );
  if ( data.resize( 4 ); !read( SeekDeviceCommand::GET_ERROR_CODE, data ) )
    throw SeekSetupError( "Failed to read error code!" );
  if ( data[0] != 0x00 && data[1] != 0x00 && data[2] != 0x00 && data[3] != 0x00 )
    throw SeekSetupError( "Camera reported error code during setup: " + data_to_string( data ) );
  if ( !write( SeekDeviceCommand::TOGGLE_SHUTTER, { 0xFC, 0x00, 0x04, 0x00 } ) )
    throw SeekSetupError( "Failed to toggle shutter!" );
  retries = 0;
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
