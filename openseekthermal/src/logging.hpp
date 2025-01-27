// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef OPENSEEKTHERMAL_LOGGING_HPP
#define OPENSEEKTHERMAL_LOGGING_HPP

#include <iomanip>
#include <sstream>
#include <vector>

#if DISABLE_LOGGING

#define LOG_DEBUG( ... )                                                                           \
  do { /* nothing*/                                                                                \
  } while ( false )
#define LOG_WARN( ... )                                                                            \
  do { /* nothing*/                                                                                \
  } while ( false )
#define LOG_ERROR( ... )                                                                           \
  do { /* nothing*/                                                                                \
  } while ( false )

#else

#if ENABLE_DEBUG_LOGGING
#define LOG_DEBUG( ... ) std::cerr << "OpenSeekThermal - DEBUG: " << __VA_ARGS__ << std::endl
#else
#define LOG_DEBUG( ... )                                                                           \
  do { /* nothing*/                                                                                \
  } while ( false )
#endif

#define LOG_WARN( ... )                                                                            \
  std::cerr << "OpenSeekThermal - WARN: " << __VA_ARGS__ << std::endl << std::flush
#define LOG_ERROR( ... )                                                                           \
  std::cerr << "OpenSeekThermal - ERROR: " << __VA_ARGS__ << std::endl << std::flush
#endif

namespace
{
inline std::string data_to_string( const std::vector<unsigned char> &data )
{
  std::stringstream result;
  result << std::hex;
  for ( auto byte : data ) {
    result << std::setfill( '0' ) << std::setw( 2 ) << static_cast<int>( byte ) << " ";
  }
  return result.str();
}
} // namespace

#endif // OPENSEEKTHERMAL_LOGGING_HPP
