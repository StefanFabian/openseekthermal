// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "openseekthermal/detail/frame.hpp"
#include "openseekthermal/detail/exceptions.hpp"

namespace openseekthermal
{

int FrameHeader::GetFrameNumberOffset( SeekDevice::Type type )
{
  switch ( type ) {
  case SeekDevice::Type::SeekThermalCompact:
    return 80;
  case SeekDevice::Type::SeekThermalCompactPro:
  case SeekDevice::Type::SeekThermalNano300:
    return 2;
  default:
    break;
  }
  throw InvalidDeviceError( "GetFrameNumberOffset not implemented for" + to_string( type ) );
}

int FrameHeader::GetFrameTypeOffset( SeekDevice::Type type )
{
  switch ( type ) {
  case SeekDevice::Type::SeekThermalCompact:
    return 20;
  case SeekDevice::Type::SeekThermalCompactPro:
  case SeekDevice::Type::SeekThermalNano300:
    return 4;
  default:
    break;
  }
  throw InvalidDeviceError( "GetFrameTypeOffset not implemented for" + to_string( type ) );
}

size_t FrameHeader::GetMinHeaderSize( SeekDevice::Type type )
{
  switch ( type ) {
  case SeekDevice::Type::SeekThermalCompact:
    return 82;
  case SeekDevice::Type::SeekThermalCompactPro:
  case SeekDevice::Type::SeekThermalNano300:
    // Full header: rows 0-2 (row 2 = per-column dark reference, useful for
    // drift analysis). Row 3 is all zeros on thermal frames, excluded.
    return 2052;
  default:
    break;
  }
  throw InvalidDeviceError( "GetMinHeaderSize not implemented for" + to_string( type ) );
}

int FrameHeader::getFrameNumber() const
{
  if ( type_ == SeekDevice::Type::None )
    return -1;
  int offset = GetFrameNumberOffset( type_ );
  if ( static_cast<int>( data_.size() ) < offset + 2 ) {
    throw SeekRuntimeError( "Frame data too small for getFrameNumber" );
  }
  return le16toh( *reinterpret_cast<const uint16_t *>( &data_[offset] ) );
}

uint16_t FrameHeader::_getRawFrameType() const
{
  if ( type_ == SeekDevice::Type::None )
    return 1337;
  int offset = GetFrameTypeOffset( type_ );
  if ( static_cast<int>( data_.size() ) < offset + 2 ) {
    throw SeekRuntimeError( "Frame data too small for getFrameType" );
  }
  return le16toh( *reinterpret_cast<const uint16_t *>( &data_[offset] ) );
}

uint16_t FrameHeader::getCountsPer100Celsius() const
{
  // Row 1 byte 16 = transfer byte 700. Only tested for Nano 300 / Compact Pro.
  // TODO: Check where it is for Compact and Nano 200, when access to one of those
  constexpr size_t kOffset = 684 + 16;
  if ( data_.size() < kOffset + 2 )
    return 0;
  return le16toh( *reinterpret_cast<const uint16_t *>( &data_[kOffset] ) );
}

uint16_t FrameHeader::getHousingAdc() const
{
  constexpr size_t kOffset = 0x0a;
  if ( data_.size() < kOffset + 2 )
    return 0;
  return le16toh( *reinterpret_cast<const uint16_t *>( &data_[kOffset] ) );
}

FrameType FrameHeader::getFrameType() const
{
  if ( type_ == SeekDevice::Type::None )
    return FrameType::UNKNOWN;

  switch ( _getRawFrameType() ) {
  case 1:
    return FrameType::CALIBRATION_FRAME;
  case 3:
    return FrameType::THERMAL_FRAME;
  case 4:
    return FrameType::FIRST_FRAME;
  case 8:
    return FrameType::STARTUP_CALIBRATION_FRAME;
  case 6:
    return FrameType::BEFORE_CALIBRATION_FRAME;
  case 20:
    return FrameType::AFTER_CALIBRATION_FRAME;
  case 7:
    return FrameType::STARTUP_AFTER_CALIBRATION_FRAME;
  default:
    break;
  }
  return FrameType::UNKNOWN;
}

std::string to_string( FrameType type )
{
  switch ( type ) {
  case FrameType::CALIBRATION_FRAME:
    return "CALIBRATION_FRAME";
  case FrameType::THERMAL_FRAME:
    return "THERMAL_FRAME";
  case FrameType::FIRST_FRAME:
    return "FIRST_FRAME";
  case FrameType::STARTUP_CALIBRATION_FRAME:
    return "STARTUP_CALIBRATION_FRAME";
  case FrameType::BEFORE_CALIBRATION_FRAME:
    return "BEFORE_CALIBRATION_FRAME";
  case FrameType::AFTER_CALIBRATION_FRAME:
    return "AFTER_CALIBRATION_FRAME";
  case FrameType::STARTUP_AFTER_CALIBRATION_FRAME:
    return "STARTUP_AFTER_CALIBRATION_FRAME";
  case FrameType::UNKNOWN:
    return "UNKNOWN";
  }
  return "INVALID";
}
} // namespace openseekthermal
