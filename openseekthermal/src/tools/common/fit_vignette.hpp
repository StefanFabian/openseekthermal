// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

// Radial-polynomial vignette fit shared by the calibration tooling.
//
// Models lens-shading as a polynomial in r² (with r the distance from the
// jointly-fit optical axis):
//
//     vignette(x, y) = c[0] + c[1]·u + … + c[D]·u^D
//     where u = ((x - cx)² + (y - cy)²) / r²_max  (≈ [0, 1])
//
// Application at runtime (after the camera's normal FFC):
//     corrected_pixel = ffc_pixel - model + mean_model
// (subtracting mean_model preserves overall scene intensity.)

#ifndef OPENSEEKTHERMAL_TOOLS_FIT_VIGNETTE_HPP
#define OPENSEEKTHERMAL_TOOLS_FIT_VIGNETTE_HPP

#include "openseekthermal/vignette_correction.hpp"

#include <vector>

namespace openseekthermal::tools
{

struct RadialFit {
  int width = 0;
  int height = 0;
  double cx = 0.0;
  double cy = 0.0;
  double r2_max = 0.0;
  int degree = 0;
  std::vector<double> coeffs;
  double mean_model = 0.0;
};

//! Evaluate the fitted polynomial at the given pixel coordinate.
double evaluate( const RadialFit &f, double x, double y );

//! Solve A * x = b in-place by Gauss-Jordan. A is N×N row-major; b is length N.
//! Returns true on success. N is small (≤ 7), so this is fine.
bool solveLinear( std::vector<double> &A, std::vector<double> &b, int N );

/*!
 * Jointly fit (cx, cy, c[0..D]) to the averaged image. The model is nonlinear
 * in the center but linear in the coefficients, so the center is refined by
 * Gauss-Newton, alternating with a closed-form coefficient resolve, followed by
 * one robust 3·MAD refit. r²_max is fixed at the image-center corner distance².
 *
 * @param dead_mask May be null. When provided, any `true` entry is excluded
 *        from every fit pass and from the MAD computation.
 */
RadialFit fitRadialPolynomial( const std::vector<double> &avg, int width, int height, int degree,
                               bool fit_center, const std::vector<bool> *dead_mask = nullptr );

//! Convert a fit result into the serializable VignetteCorrection.
VignetteCorrection toVignetteCorrection( const RadialFit &f );

} // namespace openseekthermal::tools

#endif // OPENSEEKTHERMAL_TOOLS_FIT_VIGNETTE_HPP
