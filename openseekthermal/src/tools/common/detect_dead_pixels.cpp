// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "detect_dead_pixels.hpp"

#include <algorithm>

namespace openseekthermal::tools
{

namespace
{
// Tuning constants. These are absolute / ratio-based and scene-independent, so
// detection is stable run-to-run for a comparable capture.
constexpr double kSentinelFraction = 0.5;  //!< sentinel in >=50% of frames -> unreliable
constexpr int kLocalRadius = 16;           //!< 33x33 local neighbourhood
constexpr int kMinLocalNeighbours = 8;     //!< need this many usable neighbours to judge
constexpr double kMinLocalActivity = 80.0; //!< local stddev must exceed this (region panned)
constexpr double kStuckRatio = 0.10;       //!< pixel stddev below this fraction of local -> stuck

// A pixel whose value is unreliable for the stuck test: it read a sentinel
// (0/0xFFFF) in at least half the frames, or never produced a valid sample.
// These are inpainted by the driver at runtime and are not masked here.
bool isSentinelPixel( const RawPixelStats &s, size_t i )
{
  if ( s.valid_count[i] == 0 )
    return true;
  return s.n > 0 && static_cast<double>( s.sentinel_count[i] ) >= kSentinelFraction * s.n;
}

double medianOf( std::vector<double> &v )
{
  if ( v.empty() )
    return 0.0;
  const size_t mid = v.size() / 2;
  std::nth_element( v.begin(), v.begin() + mid, v.end() );
  return v[mid];
}
} // namespace

DeadPixelResult detectDeadPixels( const RawPixelStats &stats, int width, int height )
{
  const size_t pixel_count = static_cast<size_t>( width ) * height;
  DeadPixelResult r;
  r.mask.assign( pixel_count, 0 );
  r.min_activity = kMinLocalActivity;

  // A pixel is "usable" for local statistics if it produced a real signal.
  // Sentinel pixels are excluded from neighbour stats and counted for info, but
  // never masked (the driver inpaints them at runtime).
  std::vector<uint8_t> usable( pixel_count, 0 );
  for ( size_t i = 0; i < pixel_count; ++i ) {
    if ( isSentinelPixel( stats, i ) )
      ++r.sentinel_info;
    else
      usable[i] = 1;
  }

  // Local median stddev over each pixel's neighbourhood (usable neighbours only).
  std::vector<double> local_med( pixel_count, 0.0 );
  std::vector<double> window;
  window.reserve( ( 2 * kLocalRadius + 1 ) * ( 2 * kLocalRadius + 1 ) );
  std::vector<double> activity_samples;
  activity_samples.reserve( pixel_count );
  for ( int y = 0; y < height; ++y ) {
    const int y0 = std::max( 0, y - kLocalRadius );
    const int y1 = std::min( height - 1, y + kLocalRadius );
    for ( int x = 0; x < width; ++x ) {
      const size_t i = static_cast<size_t>( y ) * width + x;
      if ( !usable[i] )
        continue;
      const int x0 = std::max( 0, x - kLocalRadius );
      const int x1 = std::min( width - 1, x + kLocalRadius );
      window.clear();
      for ( int yy = y0; yy <= y1; ++yy ) {
        for ( int xx = x0; xx <= x1; ++xx ) {
          const size_t j = static_cast<size_t>( yy ) * width + xx;
          if ( usable[j] )
            window.push_back( stats.stddev[j] );
        }
      }
      if ( static_cast<int>( window.size() ) < kMinLocalNeighbours )
        continue;
      local_med[i] = medianOf( window );
      activity_samples.push_back( local_med[i] );
    }
  }
  r.panning_activity = medianOf( activity_samples );

  // Mask only stuck pixels: usable (non-sentinel) pixels far quieter than a
  // local neighbourhood that clearly saw motion. Usable pixels whose
  // neighbourhood did not see enough motion (too few neighbours, or local
  // activity below threshold) cannot be judged and are counted as `unjudged`
  // rather than silently assumed good.
  for ( size_t i = 0; i < pixel_count; ++i ) {
    if ( !usable[i] )
      continue;
    if ( local_med[i] < kMinLocalActivity ) {
      ++r.unjudged;
      continue;
    }
    ++r.judged;
    if ( stats.stddev[i] < kStuckRatio * local_med[i] ) {
      r.mask[i] = 255;
      ++r.total_dead;
    }
  }
  return r;
}

} // namespace openseekthermal::tools
