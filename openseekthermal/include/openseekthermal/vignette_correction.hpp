// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef OPENSEEKTHERMAL_VIGNETTE_CORRECTION_HPP
#define OPENSEEKTHERMAL_VIGNETTE_CORRECTION_HPP

#include <cstdint>
#include <vector>

namespace openseekthermal
{

/*!
 * Radial polynomial vignette correction matching the model fit by the
 * calibrate_vignette tool. The model is evaluated as
 *     vignette(x, y) = sum_k coeffs[k] * u^k     with u = r²/r²_max
 * and applied as:
 *     corrected = ffc_pixel - model + mean_model
 * which preserves overall scene intensity while removing the radial trend.
 *
 * Loaded as part of a CameraCalibration; see camera_calibration.hpp.
 */
class VignetteCorrection
{
public:
  VignetteCorrection() = default;

  int width = 0;
  int height = 0;
  double cx = 0.0;
  double cy = 0.0;
  double r2_max = 0.0;
  int degree = 0;
  double mean_model = 0.0;
  std::vector<double> coeffs;

  /*!
   * Apply the correction in-place to a width*height host-endian uint16
   * frame buffer. Output is clamped to [0, 0xFFFF].
   */
  void apply( uint16_t *frame ) const;

  /*!
   * Evaluate the polynomial model at the given pixel coordinate. Useful for
   * tooling that wants to subtract the radial component from a non-uint16
   * working buffer (e.g. floating-point statistics).
   */
  double evaluate( double x, double y ) const;
};

} // namespace openseekthermal

#endif // OPENSEEKTHERMAL_VIGNETTE_CORRECTION_HPP
