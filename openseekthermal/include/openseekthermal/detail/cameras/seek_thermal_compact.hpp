// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef OPENSEEKTHERMAL_SEEK_THERMAL_COMPACT_HPP
#define OPENSEEKTHERMAL_SEEK_THERMAL_COMPACT_HPP

#include "./seek_thermal_camera.hpp"

namespace openseekthermal
{
class SeekThermalCompact : public SeekThermalCamera
{
public:
  explicit SeekThermalCompact( const SeekDevice &device, libusb_context *usb_context = nullptr );

  ~SeekThermalCompact() override;

protected:
  void setupCamera() override;
};
} // namespace openseekthermal

#endif // OPENSEEKTHERMAL_SEEK_THERMAL_COMPACT_HPP
