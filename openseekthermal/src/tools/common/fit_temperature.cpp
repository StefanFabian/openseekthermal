// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "fit_temperature.hpp"

#include <algorithm>
#include <endian.h>
#include <vector>

namespace openseekthermal::tools
{

bool accumulateAnchorFrame( const uint16_t *pixels, int row_step_pixels, int rx, int ry, int rw,
                            int rh, double robust_percentile, bool is_hot, double &sum_pct,
                            double &sum_mean, int &kept )
{
  // Per-frame robust statistic: pick the P-th percentile (P = robust_percentile
  // for cold, 100 - robust_percentile for hot). Picks pixel values closer to
  // the unvignetted optical center than a plain mean would, since vignette
  // pulls outer-ROI pixels toward the 0x4000 baseline.
  const double pct = is_hot ? ( 100.0 - robust_percentile ) : robust_percentile;

  std::vector<uint16_t> scratch;
  scratch.reserve( static_cast<size_t>( rw ) * static_cast<size_t>( rh ) );
  double frame_sum = 0.0;
  for ( int y = ry; y < ry + rh; ++y ) {
    for ( int x = rx; x < rx + rw; ++x ) {
      const uint16_t v = le16toh( pixels[y * row_step_pixels + x] );
      if ( v == 0 || v == 0xffff )
        continue;
      scratch.push_back( v );
      frame_sum += v;
    }
  }
  if ( scratch.empty() )
    return false;
  // nth_element gives us the percentile without a full sort.
  const size_t idx =
      std::min( scratch.size() - 1, static_cast<size_t>( pct / 100.0 * ( scratch.size() - 1 ) ) );
  std::nth_element( scratch.begin(), scratch.begin() + idx, scratch.end() );
  sum_pct += static_cast<double>( scratch[idx] );
  sum_mean += frame_sum / static_cast<double>( scratch.size() );
  ++kept;
  return true;
}

TwoPointAnchor finalizeAnchor( double sum_pct, double sum_mean, int kept, double robust_percentile,
                               bool is_hot )
{
  TwoPointAnchor a;
  a.percentile_used = is_hot ? ( 100.0 - robust_percentile ) : robust_percentile;
  if ( kept > 0 ) {
    a.anchor_raw = sum_pct / static_cast<double>( kept );
    a.anchor_raw_mean = sum_mean / static_cast<double>( kept );
  }
  a.thermal_used = kept;
  return a;
}

TwoPointFit solveTwoPoint( double raw_cold, double t_cold, double raw_hot, double t_hot, int x_ref )
{
  const double slope = ( t_hot - t_cold ) / ( raw_hot - raw_cold );
  // Fit math uses `T = offset_x + slope*(raw - x_ref)` for numerical
  // conditioning, then absorbs x_ref into the returned c0.
  const double offset_x = t_cold - slope * ( raw_cold - static_cast<double>( x_ref ) );
  const double offset = offset_x - slope * static_cast<double>( x_ref );
  TwoPointFit fit;
  fit.c0 = offset;
  fit.c1 = slope;
  return fit;
}

double altitudeBoilingPointC( double altitude_m ) { return 100.0 - 0.0028 * altitude_m; }

} // namespace openseekthermal::tools
