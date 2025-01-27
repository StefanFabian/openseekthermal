// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef OPENSEEKTHERMAL_SEEK_DEVICE_HPP
#define OPENSEEKTHERMAL_SEEK_DEVICE_HPP

#include "../framerate.hpp"
#include <cstdint>
#include <iostream>
#include <vector>

namespace openseekthermal
{
struct SeekDevice {
  enum class Type {
    None = 0, // For non-seek devices
    SeekThermalCompact = 1,
    SeekThermalCompactPro = 2,
    SeekThermalNano200 = 4, // Currently not supported, as I don't have one available
    SeekThermalNano300 = 8,
    All = SeekThermalCompact | SeekThermalCompactPro | SeekThermalNano200 | SeekThermalNano300
  };

  Type type = Type::None;
  //! The serial number of the device. Not all series have one. If specified takes precedence over port.
  //! Confirmed to have one: Nano 300
  //! Confirmed not to have one: Compact, CompactPro, CompactPro FF
  std::string serial;
  //! The usb address of the device. This is the bus number and port number(s). If serial is specified this is ignored.
  //! Bus and port numbers are separated by hyphen. Port numbers are separated by dots.
  //! E.g. 1-1 for bus 1, port 1 or if using a hub 1-3.2 for bus 1, hub on port 3 and port 2 on hub.
  std::string usb_port;

  //! Get the width of a frame in pixels
  int getFrameWidth() const;

  //! Get the height of a frame in pixels
  int getFrameHeight() const;

  //! Get the max framerate of the camera
  Framerate getMaxFramerate() const;

  uint16_t _getVendorID() const;

  uint16_t _getProductID() const;

  //! Get the total size of a frame to request from the device.
  //! E.g. for the nano that is half the actual total size due to the transferred data being uint16.
  uint32_t _getFrameTransferDeviceRequestSize() const;

  //! Get the total size of a frame transfer in bytes. This includes the frame and row headers.
  int _getFrameTransferTotalSize() const;

  //! Get the size of one transfer request in bytes. One frame is usually sent over multiple transfers.
  int _getFrameTransferRequestSize() const;

  //! Get the size of the frame header in bytes.
  int _getFrameHeaderSize() const;

  //! Get the size of the frame row header in bytes.
  int _getRowStep() const;

  bool _isCalibrationFrame( const std::vector<unsigned char> &buffer ) const;
};

inline SeekDevice::Type operator|( SeekDevice::Type a, SeekDevice::Type b )
{
  return static_cast<SeekDevice::Type>( static_cast<int>( a ) | static_cast<int>( b ) );
}

inline SeekDevice::Type operator&( SeekDevice::Type a, SeekDevice::Type b )
{
  return static_cast<SeekDevice::Type>( static_cast<int>( a ) & static_cast<int>( b ) );
}

enum class SeekDeviceCommand {
  GET_ERROR_CODE = 53,                  // 0x35
  READ_CHIP_ID = 54,                    // 0x36
  TOGGLE_SHUTTER = 55,                  // 0x37
  SET_SHUTTER_POLARITY = 56,            // 0x38
  GET_SHUTTER_POLARITY = 57,            // 0x39
  SET_BIT_DATA_OFFSET = 58,             // 0x3A
  GET_BIT_DATA = 59,                    // 0x3B
  SET_OPERATION_MODE = 60,              // 0x3C
  GET_OPERATION_MODE = 61,              // 0x3D
  SET_IMAGE_PROCESSING_MODE = 62,       // 0x3E
  GET_IMAGE_PROCESSING_MODE = 63,       // 0x3F
  SET_DATA_PAGE = 64,                   // 0x40
  GET_DATA_PAGE = 65,                   // 0x41
  SET_CURRENT_COMMAND_ARRAY_SIZE = 66,  // 0x42
  SET_CURRENT_COMMAND_ARRAY = 67,       // 0x43
  GET_CURRENT_COMMAND_ARRAY = 68,       // 0x44
  SET_DEFAULT_COMMAND_ARRAY_SIZE = 69,  // 0x45
  SET_DEFAULT_COMMAND_ARRAY = 70,       // 0x46
  GET_DEFAULT_COMMAND_ARRAY = 71,       // 0x47
  SET_VDAC_ARRAY_OFFSET_AND_ITEMS = 72, // 0x48
  SET_VDAC_ARRAY = 73,                  // 0x49
  GET_VDAC_ARRAY = 74,                  // 0x4A
  SET_RDAC_ARRAY_OFFSET_AND_ITEMS = 75, // 0x4B
  SET_RDAC_ARRAY = 76,                  // 0x4C
  GET_RDAC_ARRAY = 77,                  // 0x4D
  GET_FIRMWARE_INFO = 78,               // 0x4E
  UPLOAD_FIRMWARE_ROW_SIZE = 79,        // 0x4F
  WRITE_MEMORY_DATA = 80,               // 0x50
  COMPLETE_MEMORY_WRITE = 81,           // 0x51
  BEGIN_MEMORY_WRITE = 82,              // 0x52
  START_GET_IMAGE_TRANSFER = 83,        // 0x53
  TARGET_PLATFORM = 84,                 // 0x54
  SET_FIRMWARE_INFO_FEATURES = 85,      // 0x55
  SET_FACTORY_SETTINGS_FEATURES = 86,   // 0x56
  SET_FACTORY_SETTINGS = 87,            // 0x57
  GET_FACTORY_SETTINGS = 88,            // 0x58
  RESET_DEVICE = 89,                    // 0x59
};

std::string to_string( openseekthermal::SeekDevice::Type type );
std::string to_string( openseekthermal::SeekDeviceCommand command );
} // namespace openseekthermal

std::ostream &operator<<( std::ostream &os, const openseekthermal::SeekDevice &device );

#endif // OPENSEEKTHERMAL_SEEK_DEVICE_HPP
