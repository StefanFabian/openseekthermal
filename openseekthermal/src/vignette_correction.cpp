// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "openseekthermal/vignette_correction.hpp"

#include <algorithm>

namespace openseekthermal
{

double VignetteCorrection::evaluate( double x, double y ) const
{
  const double dx = x - cx;
  const double dy = y - cy;
  const double u = ( dx * dx + dy * dy ) / r2_max;
  double model = 0.0;
  double up = 1.0;
  for ( double c : coeffs ) {
    model += c * up;
    up *= u;
  }
  return model;
}

void VignetteCorrection::apply( uint16_t *frame ) const
{
  if ( coeffs.empty() || r2_max <= 0.0 )
    return;
  const int N = static_cast<int>( coeffs.size() );
  for ( int y = 0; y < height; ++y ) {
    for ( int x = 0; x < width; ++x ) {
      const double dx = x - cx;
      const double dy = y - cy;
      const double u = ( dx * dx + dy * dy ) / r2_max;
      double model = 0.0;
      double up = 1.0;
      for ( int k = 0; k < N; ++k ) {
        model += coeffs[k] * up;
        up *= u;
      }
      const size_t i = static_cast<size_t>( y ) * width + x;
      const double corrected = static_cast<double>( frame[i] ) - model + mean_model;
      const double clamped = std::clamp( corrected, 0.0, 65535.0 );
      frame[i] = static_cast<uint16_t>( clamped );
    }
  }
}

} // namespace openseekthermal
