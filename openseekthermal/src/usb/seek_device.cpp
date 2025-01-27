// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
#include "openseekthermal/detail/usb/seek_device.hpp"
#include "openseekthermal/detail/exceptions.hpp"
#include <iomanip>

std::ostream &operator<<( std::ostream &os, const openseekthermal::SeekDevice &device )
{
  if ( device.type == openseekthermal::SeekDevice::Type::None ) {
    os << "Invalid";
    return os;
  }
  switch ( device.type ) {
  case openseekthermal::SeekDevice::Type::SeekThermalNano300:
    os << "SeekThermal Nano300";
    break;
  case openseekthermal::SeekDevice::Type::SeekThermalCompactPro:
    os << "SeekThermal CompactPro";
    break;
  case openseekthermal::SeekDevice::Type::SeekThermalCompact:
    os << "SeekThermal Compact";
    break;
  default:
    os << "Invalid";
    return os;
  }
  os << " (Serial: " << device.serial << ", Port: " << device.usb_port << ")";
  return os;
}

namespace openseekthermal
{
int SeekDevice::getFrameWidth() const
{
  switch ( type ) {
  case Type::SeekThermalCompact:
    return 206; // 208 with padding
  case Type::SeekThermalNano300:
  case Type::SeekThermalCompactPro:
    return 320; // 342 with padding, 320 without
  default:
    break;
  }
  throw InvalidDeviceError( "getFrameWidth not implemented for" + to_string( type ) );
}

int SeekDevice::getFrameHeight() const
{
  switch ( type ) {
  case Type::SeekThermalCompact:
    return 156; // 156 with padding
  case Type::SeekThermalNano300:
  case Type::SeekThermalCompactPro:
    return 240; // 260 with padding, 240 without
  default:
    break;
  }
  throw InvalidDeviceError( "getFrameHeight not implemented for" + to_string( type ) );
}

Framerate SeekDevice::getMaxFramerate() const
{
  switch ( type ) {
  case Type::SeekThermalCompact:
    return 8;
  case Type::SeekThermalNano300:
    return 25;
  case Type::SeekThermalCompactPro:
    return 15;
  default:
    break;
  }
  throw InvalidDeviceError( "getMaxFramerate not implemented for" + to_string( type ) );
}

uint16_t SeekDevice::_getVendorID() const
{
  if ( type == Type::None || type == Type::All )
    throw InvalidDeviceError( "Invalid device type" );
  return 0x289d;
}

uint16_t SeekDevice::_getProductID() const
{
  {
    switch ( type ) {
    case Type::SeekThermalCompact:
      return 0x0010;
    case Type::SeekThermalCompactPro:
    case Type::SeekThermalNano300:
      return 0x0011;
    default:
      break;
    }
    throw InvalidDeviceError( "_getProductID not implemented for" + to_string( type ) );
  }
}

int SeekDevice::_getFrameTransferTotalSize() const
{
  switch ( type ) {
  case Type::SeekThermalCompact:
    return 64896; // 208 * 156 * 2
  case Type::SeekThermalCompactPro:
  case Type::SeekThermalNano300:
    return 177840; // 260 * 342 * 2
  default:
    break;
  }
  throw InvalidDeviceError( "_getFrameTransferTotalSize not implemented for" + to_string( type ) );
}

int SeekDevice::_getFrameTransferRequestSize() const
{
  switch ( type ) {
  case Type::SeekThermalCompact:
    return 16224;
  case Type::SeekThermalCompactPro:
  case Type::SeekThermalNano300:
    return 13680;
  default:
    break;
  }
  throw InvalidDeviceError( "_getFrameTransferRequestSize not implemented for" + to_string( type ) );
}

int SeekDevice::_getFrameHeaderSize() const
{
  switch ( type ) {
  case Type::SeekThermalCompact:
    return 0;
  case Type::SeekThermalCompactPro:
  case Type::SeekThermalNano300:
    return 2736; // 342 * 4 * 2
  default:
    break;
  }
  throw InvalidDeviceError( "_getFrameHeaderSize not implemented for" + to_string( type ) );
}

uint32_t SeekDevice::_getFrameTransferDeviceRequestSize() const
{
  switch ( type ) {
  case Type::SeekThermalCompact:
    return 32448; // 208 * 156
  case Type::SeekThermalCompactPro:
  case Type::SeekThermalNano300:
    return 88920; // 260 * 342
  default:
    break;
  }
  throw InvalidDeviceError( "_getFrameTransferDeviceRequestSize not implemented for" +
                            to_string( type ) );
}

int SeekDevice::_getRowStep() const
{
  switch ( type ) {
  case Type::SeekThermalCompact:
    return 416; // 208 * 2 bytes per pixel
  case Type::SeekThermalCompactPro:
  case Type::SeekThermalNano300:
    return 684; // 342 * 2 bytes per pixel
  default:
    break;
  }
  throw InvalidDeviceError( "_getRowStep not implemented for" + to_string( type ) );
}

bool SeekDevice::_isCalibrationFrame( const std::vector<unsigned char> &buffer ) const
{
  switch ( type ) {
  case Type::SeekThermalCompact:
  case Type::SeekThermalCompactPro:
  case Type::SeekThermalNano300: {
    if ( buffer.size() < 5 ) {
      return false;
    }
    auto *data = reinterpret_cast<const uint16_t *>( buffer.data() );
    return le16toh( data[2] ) == 1;
  }
  default:
    break;
  }
  throw InvalidDeviceError( "_isCalibrationFrame not implemented for" + to_string( type ) );
}

std::string to_string( SeekDevice::Type type )
{
  switch ( type ) {
  case SeekDevice::Type::SeekThermalCompact:
    return "SeekThermalCompact";
  case SeekDevice::Type::SeekThermalCompactPro:
    return "SeekThermalCompactPro";
  case SeekDevice::Type::SeekThermalNano200:
    return "SeekThermalNano200";
  case SeekDevice::Type::SeekThermalNano300:
    return "SeekThermalNano300";
  case SeekDevice::Type::None:
    return "None";
  case SeekDevice::Type::All:
    return "All";
  }
  return "INVALID";
}

std::string to_string( SeekDeviceCommand command )
{
  switch ( command ) {
  case SeekDeviceCommand::GET_ERROR_CODE:
    return "GET_ERROR_CODE";
  case SeekDeviceCommand::READ_CHIP_ID:
    return "READ_CHIP_ID";
  case SeekDeviceCommand::TOGGLE_SHUTTER:
    return "TOGGLE_SHUTTER";
  case SeekDeviceCommand::SET_OPERATION_MODE:
    return "SET_OPERATION_MODE";
  case SeekDeviceCommand::GET_OPERATION_MODE:
    return "GET_OPERATION_MODE";
  case SeekDeviceCommand::SET_IMAGE_PROCESSING_MODE:
    return "SET_IMAGE_PROCESSING_MODE";
  case SeekDeviceCommand::GET_FIRMWARE_INFO:
    return "GET_FIRMWARE_INFO";
  case SeekDeviceCommand::START_GET_IMAGE_TRANSFER:
    return "START_GET_IMAGE_TRANSFER";
  case SeekDeviceCommand::TARGET_PLATFORM:
    return "TARGET_PLATFORM";
  case SeekDeviceCommand::SET_FIRMWARE_INFO_FEATURES:
    return "SET_FIRMWARE_INFO_FEATURES";
  case SeekDeviceCommand::SET_FACTORY_SETTINGS_FEATURES:
    return "SET_FACTORY_SETTINGS_FEATURES";
  case SeekDeviceCommand::GET_FACTORY_SETTINGS:
    return "GET_FACTORY_SETTINGS";
  case SeekDeviceCommand::RESET_DEVICE:
    return "RESET_DEVICE";
  case SeekDeviceCommand::SET_SHUTTER_POLARITY:
    return "SET_SHUTTER_POLARITY";
  case SeekDeviceCommand::GET_SHUTTER_POLARITY:
    return "GET_SHUTTER_POLARITY";
  case SeekDeviceCommand::SET_BIT_DATA_OFFSET:
    return "SET_BIT_DATA_OFFSET";
  case SeekDeviceCommand::GET_BIT_DATA:
    return "GET_BIT_DATA";
  case SeekDeviceCommand::GET_IMAGE_PROCESSING_MODE:
    return "GET_IMAGE_PROCESSING_MODE";
  case SeekDeviceCommand::SET_DATA_PAGE:
    return "SET_DATA_PAGE";
  case SeekDeviceCommand::GET_DATA_PAGE:
    return "GET_DATA_PAGE";
  case SeekDeviceCommand::SET_CURRENT_COMMAND_ARRAY_SIZE:
    return "SET_CURRENT_COMMAND_ARRAY_SIZE";
  case SeekDeviceCommand::SET_CURRENT_COMMAND_ARRAY:
    return "SET_CURRENT_COMMAND_ARRAY";
  case SeekDeviceCommand::GET_CURRENT_COMMAND_ARRAY:
    return "GET_CURRENT_COMMAND_ARRAY";
  case SeekDeviceCommand::SET_DEFAULT_COMMAND_ARRAY_SIZE:
    return "SET_DEFAULT_COMMAND_ARRAY_SIZE";
  case SeekDeviceCommand::SET_DEFAULT_COMMAND_ARRAY:
    return "SET_DEFAULT_COMMAND_ARRAY";
  case SeekDeviceCommand::GET_DEFAULT_COMMAND_ARRAY:
    return "GET_DEFAULT_COMMAND_ARRAY";
  case SeekDeviceCommand::SET_VDAC_ARRAY_OFFSET_AND_ITEMS:
    return "SET_VDAC_ARRAY_OFFSET_AND_ITEMS";
  case SeekDeviceCommand::SET_VDAC_ARRAY:
    return "SET_VDAC_ARRAY";
  case SeekDeviceCommand::GET_VDAC_ARRAY:
    return "GET_VDAC_ARRAY";
  case SeekDeviceCommand::SET_RDAC_ARRAY_OFFSET_AND_ITEMS:
    return "SET_RDAC_ARRAY_OFFSET_AND_ITEMS";
  case SeekDeviceCommand::SET_RDAC_ARRAY:
    return "SET_RDAC_ARRAY";
  case SeekDeviceCommand::GET_RDAC_ARRAY:
    return "GET_RDAC_ARRAY";
  case SeekDeviceCommand::UPLOAD_FIRMWARE_ROW_SIZE:
    return "UPLOAD_FIRMWARE_ROW_SIZE";
  case SeekDeviceCommand::WRITE_MEMORY_DATA:
    return "WRITE_MEMORY_DATA";
  case SeekDeviceCommand::COMPLETE_MEMORY_WRITE:
    return "COMPLETE_MEMORY_WRITE";
  case SeekDeviceCommand::BEGIN_MEMORY_WRITE:
    return "BEGIN_MEMORY_WRITE";
  case SeekDeviceCommand::SET_FACTORY_SETTINGS:
    return "SET_FACTORY_SETTINGS";
  }
  return "INVALID";
}
} // namespace openseekthermal
