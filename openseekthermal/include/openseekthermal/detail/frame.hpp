// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef OPENSEEKTHERMAL_FRAME_HPP
#define OPENSEEKTHERMAL_FRAME_HPP

#include "./usb/seek_device.hpp"
#include <array>

namespace openseekthermal
{

enum class FrameType {
  CALIBRATION_FRAME,
  THERMAL_FRAME,
  FIRST_FRAME,
  BEFORE_CALIBRATION_FRAME,
  AFTER_CALIBRATION_FRAME,
  UNKNOWN
};

std::string to_string( FrameType type );

struct FrameHeader {
public:
  explicit FrameHeader( SeekDevice::Type type = SeekDevice::Type::None,
                        std::vector<unsigned char> data = {} )
      : type_( type ), data_( data )
  {
  }

  int getFrameNumber() const;

  FrameType getFrameType() const;

  const std::vector<unsigned char> &data() const { return data_; }

  uint16_t _getRawFrameType() const;

  static int GetFrameNumberOffset( SeekDevice::Type type );
  static int GetFrameTypeOffset( SeekDevice::Type type );
  static size_t GetMinHeaderSize( SeekDevice::Type type );

private:
  SeekDevice::Type type_;
  std::vector<unsigned char> data_;

  friend class SeekThermalCamera;
};
} // namespace openseekthermal

#endif // OPENSEEKTHERMAL_FRAME_HPP
