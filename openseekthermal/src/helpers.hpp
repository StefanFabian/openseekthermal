// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef OPENSEEKTHERMAL_HELPERS_HPP
#define OPENSEEKTHERMAL_HELPERS_HPP

#include "./logging.hpp"
#include <string>

inline std::string get_usb_descriptor_ascii_string( libusb_device_handle *handle, uint8_t index )
{
  if ( index == 0 )
    return {};
  unsigned char buffer[256];
  int length = libusb_get_string_descriptor_ascii( handle, index, buffer, sizeof( buffer ) );
  if ( length < 0 ) {
    LOG_WARN( "Failed to get string descriptor at index" << index << ": "
                                                         << libusb_error_name( length ) );
    return {};
  }
  return std::string( reinterpret_cast<const char *>( buffer ), length );
}

inline std::string bus_and_port_numbers_to_string( libusb_device *device )
{
  uint8_t port_numbers[7]; // 7 is current maximum as per USB 3.0 spec according to libusb
  int port_count = libusb_get_port_numbers( device, port_numbers, sizeof( port_numbers ) );
  std::string port;
  if ( port_count == 0 ) {
    return port;
  }
  std::stringstream ss;
  ss << static_cast<int>( libusb_get_bus_number( device ) ) << "-";
  for ( int i = 0; i < port_count; ++i ) { ss << static_cast<int>( port_numbers[i] ) << "."; }
  port = std::move( ss ).str();
  // Remove last divider
  port.resize( port.size() - 1 );
  return port;
}

#endif // OPENSEEKTHERMAL_HELPERS_HPP
