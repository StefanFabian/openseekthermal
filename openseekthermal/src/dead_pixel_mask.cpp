// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "openseekthermal/dead_pixel_mask.hpp"

#include <stdexcept>
#include <string>
#include <vector>

namespace openseekthermal
{

namespace
{

// Same 3x3 weights as extractFrame's stuck-pixel inpainter; centre weight 4
// is unused here because the dead pixel itself is never a neighbour of itself.
constexpr int kFilterWeights[9] = { 1, 2, 1, 2, 4, 2, 1, 2, 1 };

} // namespace

DeadPixelMask::DeadPixelMask( int width, int height, const std::vector<bool> &mask )
    : width_( width ), height_( height )
{
  const size_t pixel_count = static_cast<size_t>( width ) * height;
  if ( mask.size() != pixel_count ) {
    throw std::invalid_argument( "DeadPixelMask: mask size does not match width*height" );
  }
  for ( int y = 0; y < height; ++y ) {
    for ( int x = 0; x < width; ++x ) {
      const size_t idx = static_cast<size_t>( y ) * width + x;
      if ( !mask[idx] )
        continue;
      DeadPixelEntry entry;
      entry.index = static_cast<uint32_t>( idx );
      entry.neighbor_count = 0;
      for ( int k = -1; k <= 1; ++k ) {
        for ( int m = -1; m <= 1; ++m ) {
          if ( k == 0 && m == 0 )
            continue;
          const int ny = y + k;
          const int nx = x + m;
          if ( ny < 0 || ny >= height || nx < 0 || nx >= width )
            continue;
          const size_t nidx = static_cast<size_t>( ny ) * width + nx;
          if ( mask[nidx] )
            continue;
          entry.neighbors[entry.neighbor_count] = static_cast<uint32_t>( nidx );
          entry.weights[entry.neighbor_count] =
              static_cast<uint8_t>( kFilterWeights[( k + 1 ) * 3 + ( m + 1 )] );
          ++entry.neighbor_count;
        }
      }
      entries_.push_back( entry );
    }
  }
}

DeadPixelMask::DeadPixelMask( int width, int height,
                              const std::vector<std::pair<int, int>> &dead_coords )
{
  const size_t pixel_count = static_cast<size_t>( width ) * height;
  std::vector<bool> mask( pixel_count, false );
  for ( const auto &[x, y] : dead_coords ) {
    if ( x < 0 || x >= width || y < 0 || y >= height ) {
      throw std::out_of_range( "DeadPixelMask: pixel (" + std::to_string( x ) + ", " +
                               std::to_string( y ) + ") outside " + std::to_string( width ) + "x" +
                               std::to_string( height ) );
    }
    mask[static_cast<size_t>( y ) * width + x] = true;
  }
  *this = DeadPixelMask( width, height, mask );
}

void DeadPixelMask::apply( uint16_t *frame ) const
{
  for ( const DeadPixelEntry &entry : entries_ ) {
    if ( entry.neighbor_count == 0 )
      continue;
    int sum = 0;
    int total_weight = 0;
    for ( uint8_t i = 0; i < entry.neighbor_count; ++i ) {
      const int v = frame[entry.neighbors[i]];
      const int w = entry.weights[i];
      sum += v * w;
      total_weight += w;
    }
    // cppcheck-suppress zerodiv ; neighbor_count > 0 (checked above) and weights are > 0
    frame[entry.index] = static_cast<uint16_t>( sum / total_weight );
  }
}

} // namespace openseekthermal
