// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef OPENSEEKTHERMAL_FRAME_HPP
#define OPENSEEKTHERMAL_FRAME_HPP

#include "./usb/seek_device.hpp"
#include <vector>

namespace openseekthermal
{

enum class FrameType {
  CALIBRATION_FRAME,
  THERMAL_FRAME,
  FIRST_FRAME,
  STARTUP_CALIBRATION_FRAME,
  BEFORE_CALIBRATION_FRAME,
  AFTER_CALIBRATION_FRAME,
  STARTUP_AFTER_CALIBRATION_FRAME,
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

  //! Per-unit firmware-encoded raw-count delta that the sensor traverses over
  //! a 100 °C scene change. Read from row 1 byte 16 of the header (transfer
  //! byte 700). The temperature slope follows as
  //! c1 [°C/count] = 100 / getCountsPer100Celsius(). Returns 0 if the header
  //! is too short.
  uint16_t getCountsPer100Celsius() const;

  //! Housing NTC ADC reading (row 0, byte offset 0x0a). Per-frame thermometer
  //! reading of the camera body temperature. Returns 0 if the header is too
  //! short.
  uint16_t getHousingAdc() const;

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
