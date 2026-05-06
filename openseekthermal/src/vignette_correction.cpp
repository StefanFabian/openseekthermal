// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "openseekthermal/vignette_correction.hpp"

#include <algorithm>
#include <cctype>
#include <endian.h>
#include <fstream>
#include <stdexcept>
#include <unordered_map>

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
      const double corrected = static_cast<double>( le16toh( frame[i] ) ) - model + mean_model;
      const double clamped = std::clamp( corrected, 0.0, 65535.0 );
      frame[i] = htole16( static_cast<uint16_t>( clamped ) );
    }
  }
}

VignetteCorrection loadVignetteCorrection( const std::filesystem::path &path, int expected_width,
                                           int expected_height )
{
  std::ifstream in( path );
  if ( !in ) {
    throw std::runtime_error( "Could not open vignette correction: " + path.string() );
  }
  // Minimal INI parser: blank lines and lines starting with '#' or ';' are
  // skipped; '[section]' headers are accepted but not required (we don't
  // distinguish sections — every key collapses into one map). Lines must be
  // of the form `key = value`. Trailing inline `;` or `#` comments are stripped.
  auto trim = []( std::string &s ) {
    size_t a = 0;
    while ( a < s.size() && std::isspace( static_cast<unsigned char>( s[a] ) ) ) ++a;
    size_t b = s.size();
    while ( b > a && std::isspace( static_cast<unsigned char>( s[b - 1] ) ) ) --b;
    s = s.substr( a, b - a );
  };
  std::unordered_map<std::string, std::string> kv;
  std::string line;
  int line_no = 0;
  while ( std::getline( in, line ) ) {
    ++line_no;
    // Strip inline comments (anywhere a `;` or `#` appears at start-of-token).
    for ( size_t i = 0; i < line.size(); ++i ) {
      if ( line[i] == ';' || line[i] == '#' ) {
        line.resize( i );
        break;
      }
    }
    trim( line );
    if ( line.empty() )
      continue;
    if ( line.front() == '[' && line.back() == ']' )
      continue; // section header — ignored
    const size_t eq = line.find( '=' );
    if ( eq == std::string::npos ) {
      throw std::runtime_error( "Vignette file " + path.string() + ": malformed line " +
                                std::to_string( line_no ) + " (expected `key = value`)" );
    }
    std::string key = line.substr( 0, eq );
    std::string value = line.substr( eq + 1 );
    trim( key );
    trim( value );
    if ( key.empty() ) {
      throw std::runtime_error( "Vignette file " + path.string() + ": empty key on line " +
                                std::to_string( line_no ) );
    }
    kv[key] = value;
  }

  auto require = [&]( const std::string &key ) -> const std::string & {
    auto it = kv.find( key );
    if ( it == kv.end() ) {
      throw std::runtime_error( "Vignette file missing required key '" + key + "': " + path.string() );
    }
    return it->second;
  };
  auto requireInt = [&]( const std::string &key ) -> int {
    const std::string &s = require( key );
    try {
      return std::stoi( s );
    } catch ( const std::exception & ) {
      throw std::runtime_error( "Vignette file " + path.string() + ": key '" + key +
                                "' has invalid integer value '" + s + "'" );
    }
  };
  auto requireDouble = [&]( const std::string &key ) -> double {
    const std::string &s = require( key );
    try {
      return std::stod( s );
    } catch ( const std::exception & ) {
      throw std::runtime_error( "Vignette file " + path.string() + ": key '" + key +
                                "' has invalid floating-point value '" + s + "'" );
    }
  };

  VignetteCorrection v;
  v.width = requireInt( "width" );
  v.height = requireInt( "height" );
  v.cx = requireDouble( "center_x" );
  v.cy = requireDouble( "center_y" );
  v.r2_max = requireDouble( "r2_max" );
  v.degree = requireInt( "degree" );
  v.mean_model = requireDouble( "mean_model" );
  if ( v.width != expected_width || v.height != expected_height ) {
    throw std::runtime_error( "Vignette correction size " + std::to_string( v.width ) + "x" +
                              std::to_string( v.height ) + " does not match camera " +
                              std::to_string( expected_width ) + "x" +
                              std::to_string( expected_height ) );
  }
  if ( v.degree < 0 || v.degree > 32 ) {
    throw std::runtime_error( "Vignette degree out of range: " + std::to_string( v.degree ) );
  }
  v.coeffs.resize( static_cast<size_t>( v.degree ) + 1 );
  for ( int k = 0; k <= v.degree; ++k ) {
    v.coeffs[k] = requireDouble( "c" + std::to_string( k ) );
  }
  return v;
}

} // namespace openseekthermal
