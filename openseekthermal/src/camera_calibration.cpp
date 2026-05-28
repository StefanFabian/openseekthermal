// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "openseekthermal/camera_calibration.hpp"

#include <cctype>
#include <charconv>
#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

namespace openseekthermal
{

namespace
{

void trim( std::string &s )
{
  size_t a = 0;
  while ( a < s.size() && std::isspace( static_cast<unsigned char>( s[a] ) ) ) ++a;
  size_t b = s.size();
  while ( b > a && std::isspace( static_cast<unsigned char>( s[b - 1] ) ) ) --b;
  s = s.substr( a, b - a );
}

bool parseDoubleC( const std::string &s, double &out )
{
  const char *first = s.data();
  const char *last = s.data() + s.size();
  auto [ptr, ec] = std::from_chars( first, last, out );
  if ( ec != std::errc() )
    return false;
  while ( ptr != last && std::isspace( static_cast<unsigned char>( *ptr ) ) ) ++ptr;
  return ptr == last;
}

bool parseIntC( const std::string &s, int &out )
{
  const char *first = s.data();
  const char *last = s.data() + s.size();
  int base = 10;
  if ( last - first >= 2 && first[0] == '0' && ( first[1] == 'x' || first[1] == 'X' ) ) {
    base = 16;
    first += 2;
  }
  auto [ptr, ec] = std::from_chars( first, last, out, base );
  if ( ec != std::errc() )
    return false;
  while ( ptr != last && std::isspace( static_cast<unsigned char>( *ptr ) ) ) ++ptr;
  return ptr == last;
}

using Section = std::unordered_map<std::string, std::string>;

struct ParsedFile {
  std::unordered_map<std::string, Section> sections;
};

ParsedFile parseFile( std::istream &in, const std::string &path )
{
  ParsedFile out;
  std::string current_section;
  std::string line;
  int line_no = 0;
  while ( std::getline( in, line ) ) {
    ++line_no;
    for ( size_t i = 0; i < line.size(); ++i ) {
      if ( line[i] == ';' || line[i] == '#' ) {
        line.resize( i );
        break;
      }
    }
    trim( line );
    if ( line.empty() )
      continue;
    if ( line.front() == '[' && line.back() == ']' ) {
      current_section = line.substr( 1, line.size() - 2 );
      trim( current_section );
      out.sections[current_section]; // ensure present
      continue;
    }
    const size_t eq = line.find( '=' );
    if ( eq == std::string::npos ) {
      throw std::runtime_error( "Calibration file " + path + ": malformed line " +
                                std::to_string( line_no ) + " (expected `key = value`)" );
    }
    std::string key = line.substr( 0, eq );
    std::string value = line.substr( eq + 1 );
    trim( key );
    trim( value );
    if ( key.empty() ) {
      throw std::runtime_error( "Calibration file " + path + ": empty key on line " +
                                std::to_string( line_no ) );
    }
    if ( current_section.empty() ) {
      throw std::runtime_error( "Calibration file " + path + ": key '" + key + "' on line " +
                                std::to_string( line_no ) + " is outside any [section]" );
    }
    out.sections[current_section][key] = value;
  }
  return out;
}

double requireDouble( const Section &sec, const std::string &section_name, const std::string &key,
                      const std::string &path )
{
  auto it = sec.find( key );
  if ( it == sec.end() ) {
    throw std::runtime_error( "Calibration file " + path + " [" + section_name +
                              "] missing required key '" + key + "'" );
  }
  double v;
  if ( !parseDoubleC( it->second, v ) ) {
    throw std::runtime_error( "Calibration file " + path + " [" + section_name + "] key '" + key +
                              "' has invalid floating-point value '" + it->second + "'" );
  }
  return v;
}

int requireInt( const Section &sec, const std::string &section_name, const std::string &key,
                const std::string &path )
{
  auto it = sec.find( key );
  if ( it == sec.end() ) {
    throw std::runtime_error( "Calibration file " + path + " [" + section_name +
                              "] missing required key '" + key + "'" );
  }
  int v;
  if ( !parseIntC( it->second, v ) ) {
    throw std::runtime_error( "Calibration file " + path + " [" + section_name + "] key '" + key +
                              "' has invalid integer value '" + it->second + "'" );
  }
  return v;
}

double optionalDouble( const Section &sec, const std::string &key, double default_value )
{
  auto it = sec.find( key );
  if ( it == sec.end() )
    return default_value;
  double v;
  if ( !parseDoubleC( it->second, v ) ) {
    throw std::runtime_error( "Calibration value '" + key + "' = '" + it->second + "' invalid" );
  }
  return v;
}

TemperatureCalibration parseTemperatureSection( const Section &sec )
{
  TemperatureCalibration cal;
  cal.c0 = optionalDouble( sec, "c0", 0.0 );
  cal.c1 = optionalDouble( sec, "c1", 0.0 );
  return cal;
}

VignetteCorrection parseVignetteSection( const Section &sec, const std::string &path,
                                         int expected_width, int expected_height )
{
  VignetteCorrection v;
  v.width = requireInt( sec, "vignette", "width", path );
  v.height = requireInt( sec, "vignette", "height", path );
  v.cx = requireDouble( sec, "vignette", "cx", path );
  v.cy = requireDouble( sec, "vignette", "cy", path );
  v.r2_max = requireDouble( sec, "vignette", "r2_max", path );
  v.mean_model = requireDouble( sec, "vignette", "mean_model", path );
  if ( v.width != expected_width || v.height != expected_height ) {
    throw std::runtime_error( "Calibration file " + path + " [vignette] size " +
                              std::to_string( v.width ) + "x" + std::to_string( v.height ) +
                              " does not match camera " + std::to_string( expected_width ) + "x" +
                              std::to_string( expected_height ) );
  }
  auto it = sec.find( "coeffs" );
  if ( it == sec.end() ) {
    throw std::runtime_error( "Calibration file " + path +
                              " [vignette] missing required key 'coeffs'" );
  }
  std::istringstream iss( it->second );
  iss.imbue( std::locale::classic() );
  double c;
  while ( iss >> c ) v.coeffs.push_back( c );
  if ( v.coeffs.empty() ) {
    throw std::runtime_error( "Calibration file " + path + " [vignette] coeffs list is empty" );
  }
  v.degree = static_cast<int>( v.coeffs.size() ) - 1;
  return v;
}

DeadPixelMask parseDeadPixelsSection( const Section &sec, const std::string &path,
                                      int expected_width, int expected_height )
{
  auto it = sec.find( "pixels" );
  if ( it == sec.end() ) {
    return DeadPixelMask( expected_width, expected_height, std::vector<std::pair<int, int>>{} );
  }
  std::vector<std::pair<int, int>> coords;
  const std::string &s = it->second;
  size_t i = 0;
  while ( i < s.size() ) {
    while ( i < s.size() && std::isspace( static_cast<unsigned char>( s[i] ) ) ) ++i;
    if ( i >= s.size() )
      break;
    const size_t start = i;
    while ( i < s.size() && !std::isspace( static_cast<unsigned char>( s[i] ) ) ) ++i;
    const std::string token = s.substr( start, i - start );
    const size_t comma = token.find( ',' );
    if ( comma == std::string::npos ) {
      throw std::runtime_error( "Calibration file " + path + " [dead_pixels] expected 'x,y' got '" +
                                token + "'" );
    }
    int x, y;
    const std::string sx = token.substr( 0, comma );
    const std::string sy = token.substr( comma + 1 );
    if ( !parseIntC( sx, x ) || !parseIntC( sy, y ) ) {
      throw std::runtime_error( "Calibration file " + path +
                                " [dead_pixels] invalid coordinate pair '" + token + "'" );
    }
    coords.emplace_back( x, y );
  }
  return DeadPixelMask( expected_width, expected_height, coords );
}

void writeTemperatureSection( std::ostream &out, const TemperatureCalibration &cal )
{
  out << "[temperature]\n";
  out << std::scientific << std::setprecision( 10 );
  out << "c0 = " << cal.c0 << "\n";
  out << "c1 = " << cal.c1 << "\n";
  out << std::defaultfloat;
}

void writeVignetteSection( std::ostream &out, const VignetteCorrection &v )
{
  out << "[vignette]\n";
  out << "width = " << v.width << "\n";
  out << "height = " << v.height << "\n";
  out << std::setprecision( 9 );
  out << "cx = " << v.cx << "\n";
  out << "cy = " << v.cy << "\n";
  out << "r2_max = " << v.r2_max << "\n";
  out << "mean_model = " << v.mean_model << "\n";
  out << "coeffs =";
  for ( double c : v.coeffs ) out << " " << c;
  out << "\n";
}

void writeDeadPixelsSection( std::ostream &out, const DeadPixelMask &mask )
{
  out << "[dead_pixels]\n";
  if ( mask.entries().empty() ) {
    out << "pixels =\n";
    return;
  }
  out << "pixels =";
  const int w = mask.width();
  size_t per_line = 0;
  for ( const auto &entry : mask.entries() ) {
    const int x = static_cast<int>( entry.index ) % w;
    const int y = static_cast<int>( entry.index ) / w;
    out << " " << x << "," << y;
    if ( ++per_line == 16 ) {
      out << "\n        ";
      per_line = 0;
    }
  }
  out << "\n";
}

} // namespace

CameraCalibration loadCameraCalibration( const std::filesystem::path &path, int expected_width,
                                         int expected_height )
{
  std::ifstream in( path );
  if ( !in ) {
    throw std::runtime_error( "Could not open calibration: " + path.string() );
  }
  const std::string path_str = path.string();
  ParsedFile parsed = parseFile( in, path_str );
  CameraCalibration cal;
  if ( auto it = parsed.sections.find( "temperature" ); it != parsed.sections.end() ) {
    cal.temperature = parseTemperatureSection( it->second );
  }
  if ( auto it = parsed.sections.find( "vignette" ); it != parsed.sections.end() ) {
    cal.vignette = parseVignetteSection( it->second, path_str, expected_width, expected_height );
  }
  if ( auto it = parsed.sections.find( "dead_pixels" ); it != parsed.sections.end() ) {
    cal.dead_pixels = parseDeadPixelsSection( it->second, path_str, expected_width, expected_height );
  }
  return cal;
}

void saveCameraCalibration( const std::filesystem::path &path, const CameraCalibration &cal,
                            const std::string &header_comment )
{
  std::ofstream out( path );
  if ( !out ) {
    throw std::runtime_error( "Could not open calibration for writing: " + path.string() );
  }
  out.imbue( std::locale::classic() );
  if ( !header_comment.empty() ) {
    std::istringstream iss( header_comment );
    std::string line;
    while ( std::getline( iss, line ) ) { out << "# " << line << "\n"; }
  }
  if ( cal.temperature ) {
    writeTemperatureSection( out, *cal.temperature );
    out << "\n";
  }
  if ( cal.vignette ) {
    writeVignetteSection( out, *cal.vignette );
    out << "\n";
  }
  if ( cal.dead_pixels ) {
    writeDeadPixelsSection( out, *cal.dead_pixels );
    out << "\n";
  }
}

} // namespace openseekthermal
