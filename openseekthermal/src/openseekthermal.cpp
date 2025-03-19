// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "openseekthermal/openseekthermal.hpp"
#include "openseekthermal/detail/cameras/seek_thermal_compact.hpp"
#include "openseekthermal/detail/cameras/seek_thermal_compact_pro.hpp"
#include "openseekthermal/detail/cameras/seek_thermal_nano_300.hpp"
#include "openseekthermal/detail/exceptions.hpp"
#include <libusb-1.0/libusb.h>

#include "./helpers.hpp"
#include "./logging.hpp"

namespace openseekthermal
{

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
      return { SeekDevice::Type::SeekThermalCompact, "", port };
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

    std::string serial;
    if ( desc.iSerialNumber != 0 ) {
      serial = get_usb_descriptor_ascii_string( handle, desc.iSerialNumber );
    }

    if ( name.find( "PIR324 Thermal Camera" ) != std::string::npos ) {
      return { SeekDevice::Type::SeekThermalCompactPro, serial, port };
    }
    if ( name.find( "Nano300" ) != std::string::npos ) {
      return { SeekDevice::Type::SeekThermalNano300, serial, port };
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
