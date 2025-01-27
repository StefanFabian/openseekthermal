// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef OPENSEEKTHERMAL_OPENSEEKTHERMAL_HPP
#define OPENSEEKTHERMAL_OPENSEEKTHERMAL_HPP

#include "./detail/cameras/seek_thermal_camera.hpp"
#include "./detail/usb/seek_device.hpp"
#include <vector>

struct libusb_device;

namespace openseekthermal
{

/*!
 * List all available devices of the given type found connected to the system.
 * @param type The type of SeekThermal devices to list. Use All to list all devices.
 */
std::vector<SeekDevice> listDevices( SeekDevice::Type type = SeekDevice::Type::All );

/*!
 * Extract the SeekDevice information from the given libusb device.
 */
SeekDevice getSeekDevice( libusb_device *device );

/*!
 * Create a camera object for the given device.
 * Uses the serial first if available otherwise the usb_port.
 *
 * Note that the device still has to be opened with the open() function.
 *
 * @param device The device to create a camera for.
 * @param context The libusb context to use. If nullptr a new context will be created.
 * @return An instance of the camera for the given device type.
 */
SeekThermalCamera::SharedPtr createCamera( const SeekDevice &device,
                                           libusb_context *context = nullptr );

} // namespace openseekthermal

#endif // OPENSEEKTHERMAL_OPENSEEKTHERMAL_HPP
