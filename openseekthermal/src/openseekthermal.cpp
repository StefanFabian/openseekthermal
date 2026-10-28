// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "openseekthermal/openseekthermal.hpp"
#include "openseekthermal/detail/cameras/seek_thermal_compact.hpp"
#include "openseekthermal/detail/cameras/seek_thermal_compact_pro.hpp"
#include "openseekthermal/detail/cameras/seek_thermal_nano_300.hpp"
#include "openseekthermal/detail/exceptions.hpp"
#include <algorithm>
#include <cctype>
#include <cstring>
#include <libusb-1.0/libusb.h>

#include "./helpers.hpp"
#include "./logging.hpp"

namespace openseekthermal
{
namespace
{
struct FactoryInfo {
  //! E.g. CQ-DBAX (Nano 300) or CQ-AAAX (Compact Pro).
  std::string part_number;
  std::string serial;
};

//! Read FactoryInfo from the factory settings. Fields are empty if they could not be read.
FactoryInfo readFactoryInfo( libusb_device_handle *handle, const std::string &port )
{
  // Linux skips the interface ownership check for vendor requests, so without claiming we could
  // interleave with the factory reads of another process setting up this camera.
  int returncode = libusb_claim_interface( handle, 0 );
  if ( returncode < 0 ) {
    LOG_WARN( "Can not read factory info of device at port "
              << port << ", it is in use: " << libusb_error_name( returncode ) );
    return {};
  }
  auto read_factory_settings = [handle]( std::vector<unsigned char> feature,
                                         std::vector<unsigned char> &data ) {
    int returncode = libusb_control_transfer(
        handle, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        static_cast<uint8_t>( SeekDeviceCommand::SET_FACTORY_SETTINGS_FEATURES ), 0, 0,
        feature.data(), feature.size(), 1000 );
    if ( returncode >= 0 ) {
      returncode = libusb_control_transfer(
          handle, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
          static_cast<uint8_t>( SeekDeviceCommand::GET_FACTORY_SETTINGS ), 0, 0, data.data(),
          data.size(), 1000 );
    }
    if ( returncode < 0 ) {
      LOG_DEBUG( "Failed to read factory settings: " << libusb_error_name( returncode ) );
      return false;
    }
    data.resize( returncode );
    return true;
  };
  FactoryInfo info;
  std::vector<unsigned char> data( 16 );
  if ( read_factory_settings( { 0x08, 0x00, 0x02, 0x06, 0x00, 0x00 }, data ) ) {
    const auto *chars = reinterpret_cast<const char *>( data.data() );
    info.part_number.assign( chars, strnlen( chars, data.size() ) );
  }
  data.resize( 64 );
  if ( read_factory_settings( { 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 }, data ) && data.size() > 0x10 ) {
    // Page 0x00 holds the 12-char ASCII unit serial at offset 0x10, null-padded like the USB serial.
    info.serial.assign( reinterpret_cast<const char *>( data.data() ) + 0x10,
                        std::min<size_t>( 12, data.size() - 0x10 ) );
    info.serial.erase( std::find_if( info.serial.begin(), info.serial.end(),
                                     []( unsigned char c ) { return std::isprint( c ) == 0; } ),
                       info.serial.end() );
  }
  libusb_release_interface( handle, 0 );
  return info;
}
} // namespace

SeekDevice getSeekDevice( libusb_device *device )
{
  libusb_device_descriptor desc = {};
  int returncode = libusb_get_device_descriptor( device, &desc );
  if ( returncode < 0 ) {
    LOG_DEBUG( "Failed to get device descriptor for device " << returncode );
    return {};
  }
  if ( desc.idVendor != 0x289d ) {
    return {};
  }
  if ( desc.iProduct == 0 ) {
    LOG_WARN( "iProduct is 0 for device of correct vendor. This is unexpected." );
    return {};
  }
  libusb_device_handle *handle = nullptr;

  struct USBDeviceHandleGuard {
    ~USBDeviceHandleGuard()
    {
      if ( *handle != nullptr )
        libusb_close( *handle );
    }

    libusb_device_handle **handle;
  } usb_device_handle_guard{ &handle };

  std::string port = bus_and_port_numbers_to_string( device );
  switch ( desc.idProduct ) {
  case 0x0010: {
    // name would be PIR206 Thermal Camera
    returncode = libusb_open( device, &handle );
    if ( returncode < 0 ) {
      LOG_WARN( "Failed to open device: " << libusb_error_name( returncode ) );
      return {};
    }

    std::string name = get_usb_descriptor_ascii_string( handle, desc.iProduct );
    if ( name.empty() ) {
      return {};
    }
    if ( name.find( "PIR206 Thermal Camera" ) != std::string::npos ) {
      // The Compact does not advertise its serial over USB.
      return { SeekDevice::Type::SeekThermalCompact, readFactoryInfo( handle, port ).serial, port };
    }
    break; // TODO
  }
  case 0x0011: {
    // Could be Compact Pro or Nano
    returncode = libusb_open( device, &handle );
    if ( returncode < 0 ) {
      LOG_WARN( "Failed to open device: " << libusb_error_name( returncode ) );
      return {};
    }

    std::string name = get_usb_descriptor_ascii_string( handle, desc.iProduct );
    if ( name.empty() ) {
      return {};
    }

    // Not all units advertise their serial over USB, fall back to the factory data.
    std::string serial = get_usb_descriptor_ascii_string( handle, desc.iSerialNumber );

    if ( name.find( "Nano300" ) != std::string::npos ) {
      if ( serial.empty() )
        serial = readFactoryInfo( handle, port ).serial;
      return { SeekDevice::Type::SeekThermalNano300, serial, port };
    }
    if ( name.find( "PIR324 Thermal Camera" ) != std::string::npos ) {
      // Newer Nano 300 units advertise the Compact Pro name. The product family letter after the
      // dash in the part number tells them apart (CQ-DBAX Nano 300, CQ-AAAX Compact Pro).
      FactoryInfo info = readFactoryInfo( handle, port );
      if ( serial.empty() )
        serial = info.serial;
      if ( info.part_number.find( "-D" ) != std::string::npos ) {
        return { SeekDevice::Type::SeekThermalNano300, serial, port };
      }
      return { SeekDevice::Type::SeekThermalCompactPro, serial, port };
    }
    break;
  }
  default:
    LOG_DEBUG( "Vendor ID matches but unknown product ID: " << desc.idProduct );
    break;
  }
  return {};
}

std::vector<SeekDevice> listDevices( SeekDevice::Type types )
{
  libusb_context *context = nullptr;
#if LIBUSB_API_VERSION >= 0x0100010A
  int returncode = libusb_init_context( &context, nullptr, 0 );
#else
  int returncode = libusb_init( &context );
#endif
  if ( returncode < 0 ) {
    throw USBError( "Failed to initialize libusb!", returncode );
  }

  struct USBContextGuard {
    ~USBContextGuard() { libusb_exit( context ); }

    libusb_context *context;
  } usb_context_guard{ context };

  libusb_device **devs;
  const size_t count = libusb_get_device_list( context, &devs );
  if ( count == 0 ) {
    return {};
  }

  struct USBDeviceListGuard {
    ~USBDeviceListGuard() { libusb_free_device_list( devs, 1 ); }

    libusb_device **devs;
  } usb_device_list_guard{ devs };

  std::vector<SeekDevice> result;
  for ( size_t i = 0; i < count; ++i ) {
    libusb_device *device = devs[i];
    SeekDevice seek_device = getSeekDevice( device );
    if ( ( seek_device.type & types ) != SeekDevice::Type::None ) {
      result.push_back( seek_device );
    }
  }
  return result;
}

SeekThermalCamera::SharedPtr createCamera( const SeekDevice &device, libusb_context *context )
{
  switch ( device.type ) {
  case SeekDevice::Type::SeekThermalCompact:
    return std::make_shared<SeekThermalCompact>( device, context );
  case SeekDevice::Type::SeekThermalCompactPro:
    return std::make_shared<SeekThermalCompactPro>( device, context );
  case SeekDevice::Type::SeekThermalNano300:
    return std::make_shared<SeekThermalNano300>( device, context );
  default:
    break;
  }
  return nullptr;
}

} // namespace openseekthermal
