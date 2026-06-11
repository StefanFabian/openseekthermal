// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef OPENSEEKTHERMAL_TEMPERATURE_CALIBRATION_HPP
#define OPENSEEKTHERMAL_TEMPERATURE_CALIBRATION_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace openseekthermal
{

/*!
 * Two-point forward model mapping (shutter-corrected, drift-compensated) raw
 * counts to scene temperature in centi-Kelvin:
 *
 *     cK = clamp(round(100 * (c0 + c1*raw) + 27315), 0, 0xFFFF)
 *
 * `c0` and `c1` are the two-point linear forward fit.
 *   c1 (slope) is per-unit on-device: `c1 = 100 / u16@row1+16` (row 1 of every
 *   transfer); the driver seeds it automatically.
 *   c0 (absolute offset) is derived from the calibration reference frame, which
 *   sits at the factory reference temperature (~22°C on most units).
 *
 * The driver runs an in-band substrate-drift compensation on the raw counts
 * before applying the temperature mapping.
 */
struct TemperatureCalibration {
  double c0 = 0.0; //!< °C intercept
  double c1 = 1.0; //!< °C / raw count

  //! Pad-column drift reference the c0 fit was anchored to. Sentinel NaN =
  //! unset. When set, the driver pins its in-band drift anchor to this value
  //! instead of the per-boot first-thermal pad, keeping a fixed c0 valid across
  //! reboots. Must equal the `ref` the fit used (driver applies
  //! raw - K*(pad - pad_ref)). Left unset on the camera-only auto-c0 path, where
  //! the anchor cancels out of the final temperature and need not be pinned.
  double pad_ref = std::numeric_limits<double>::quiet_NaN();

  //! Apply to one pixel. Returns clamped uint16 cK.
  inline uint16_t apply( uint16_t raw ) const noexcept
  {
    const double t_c = c0 + c1 * static_cast<double>( raw );
    const double cK = std::round( 100.0 * t_c + 27315.0 );
    const double clamped = std::clamp( cK, 0.0, 65535.0 );
    return static_cast<uint16_t>( clamped );
  }
};

} // namespace openseekthermal

#endif // OPENSEEKTHERMAL_TEMPERATURE_CALIBRATION_HPP
