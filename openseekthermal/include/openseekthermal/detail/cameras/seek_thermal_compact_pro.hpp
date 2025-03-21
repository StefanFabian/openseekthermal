// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef OPENSEEKTHERMAL_SEEK_THERMAL_COMPACT_PRO_HPP
#define OPENSEEKTHERMAL_SEEK_THERMAL_COMPACT_PRO_HPP

#include "./seek_thermal_camera.hpp"

namespace openseekthermal
{
class SeekThermalCompactPro : public SeekThermalCamera
{
public:
  explicit SeekThermalCompactPro( const SeekDevice &device, libusb_context *usb_context = nullptr );

  ~SeekThermalCompactPro() override;

protected:
  void setupCamera() override;
};
} // namespace openseekthermal

#endif // OPENSEEKTHERMAL_SEEK_THERMAL_COMPACT_PRO_HPP
