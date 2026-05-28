// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef OPENSEEKTHERMAL_TEMPERATURE_CALIBRATION_HPP
#define OPENSEEKTHERMAL_TEMPERATURE_CALIBRATION_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace openseekthermal
{

/*!
 * Two-point forward model mapping (shutter-corrected, drift-compensated) raw
 * counts to scene temperature in centi-Kelvin:
 *
 *     cK = clamp(round(100 * (c0 + c1*raw) + 27315), 0, 0xFFFF)
 *
 * `c0` and `c1` are the two-point linear forward fit. The driver computes
 * them from on-device data when none is supplied by the host:
 *     c1 = 100 / u16@row1+16                 (per-unit, row 1 of every transfer)
 *     c0 = T_ref - c1 * raw_at_T_ref          (factory dump 0x4c, 0x44)
 *
 * The driver runs an in-band substrate-drift compensation on the raw counts
 * before calling `apply()`; calibrations should therefore be fit against the
 * compensated stream (e.g. by `calibrate_ice_boil`) without their own
 * housing-dependent terms.
 */
struct TemperatureCalibration {
  double c0 = 0.0; //!< °C intercept
  double c1 = 1.0; //!< °C / raw count

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
