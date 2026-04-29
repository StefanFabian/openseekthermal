// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "openseekthermal/dead_pixel_mask.hpp"

#include <cctype>
#include <endian.h>
#include <fstream>
#include <stdexcept>

namespace openseekthermal
{

namespace
{

// Same 3x3 weights as extractFrame's stuck-pixel inpainter; centre weight 4
// is unused here because the dead pixel itself is never a neighbour of itself.
constexpr int kFilterWeights[9] = { 1, 2, 1, 2, 4, 2, 1, 2, 1 };

void skipWhitespaceAndComments( std::istream &in )
{
  char c;
  while ( in.get( c ) ) {
    if ( c == '#' ) {
      while ( in.get( c ) && c != '\n' ) {}
    } else if ( !std::isspace( static_cast<unsigned char>( c ) ) ) {
      in.unget();
      break;
    }
  }
}

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

void DeadPixelMask::apply( uint16_t *frame ) const
{
  for ( const DeadPixelEntry &entry : entries_ ) {
    if ( entry.neighbor_count == 0 )
      continue;
    int sum = 0;
    int total_weight = 0;
    for ( uint8_t i = 0; i < entry.neighbor_count; ++i ) {
      const int v = le16toh( frame[entry.neighbors[i]] );
      const int w = entry.weights[i];
      sum += v * w;
      total_weight += w;
    }
    frame[entry.index] = htole16( static_cast<uint16_t>( sum / total_weight ) );
  }
}

DeadPixelMask loadDeadPixelMaskPgm( const std::filesystem::path &path, int expected_width,
                                    int expected_height )
{
  std::ifstream in( path, std::ios::binary );
  if ( !in ) {
    throw std::runtime_error( "Could not open dead-pixel mask: " + path.string() );
  }
  std::string magic;
  in >> magic;
  if ( magic != "P5" ) {
    throw std::runtime_error( "Dead-pixel mask is not a P5 PGM: " + path.string() );
  }
  skipWhitespaceAndComments( in );
  int w = 0;
  in >> w;
  skipWhitespaceAndComments( in );
  int h = 0;
  in >> h;
  skipWhitespaceAndComments( in );
  int maxval = 0;
  in >> maxval;
  in.get(); // consume the single byte of whitespace after maxval
  if ( maxval != 255 ) {
    throw std::runtime_error( "Dead-pixel mask must be 8-bit (maxval 255), got " +
                              std::to_string( maxval ) );
  }
  if ( w != expected_width || h != expected_height ) {
    throw std::runtime_error( "Dead-pixel mask size " + std::to_string( w ) + "x" +
                              std::to_string( h ) + " does not match camera " +
                              std::to_string( expected_width ) + "x" +
                              std::to_string( expected_height ) );
  }
  std::vector<uint8_t> bytes( static_cast<size_t>( w ) * h );
  in.read( reinterpret_cast<char *>( bytes.data() ),
           static_cast<std::streamsize>( bytes.size() ) );
  if ( !in ) {
    throw std::runtime_error( "Short read on dead-pixel mask: " + path.string() );
  }
  std::vector<bool> mask( bytes.size() );
  for ( size_t i = 0; i < bytes.size(); ++i ) mask[i] = bytes[i] != 0;
  return DeadPixelMask( w, h, mask );
}

} // namespace openseekthermal
