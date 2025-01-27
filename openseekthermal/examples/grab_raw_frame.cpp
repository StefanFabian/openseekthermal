// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "openseekthermal/openseekthermal.hpp"
#include <chrono>
#include <fstream>
#include <iomanip>

using namespace openseekthermal;

int main( int, char ** )
{
  auto devices = openseekthermal::listDevices();
  if ( devices.empty() ) {
    std::cout << "No devices found!" << std::endl;
    return 1;
  }
  std::cout << "Found " << devices.size() << " devices:" << std::endl;
  std::cout << "Using first" << std::endl;
  auto device = devices[0];
  SeekThermalCamera::SharedPtr cam = openseekthermal::createCamera( device );
  if ( cam == nullptr ) {
    std::cout << "Failed to create camera" << std::endl;
    return 1;
  }
  cam->open();
  std::cout << "Camera opened" << std::endl;
  std::cout << "Firmware info: " << cam->readFirmwareInfo() << std::endl;
  std::cout << "Chip ID: " << cam->readChipID() << std::endl;

  unsigned char *frame_data = nullptr;
  size_t size;
  while ( true ) {
    GrabFrameResult result = cam->_grabRawFrame( &frame_data, size );
    if ( result != GrabFrameResult::SUCCESS )
      continue;
    std::vector<unsigned char> data( frame_data,
                                     frame_data + FrameHeader::GetMinHeaderSize( device.type ) );
    FrameHeader header( device.type, data );
    if ( header.getFrameType() == FrameType::THERMAL_FRAME ) {
      std::ofstream frame( "frame.bin", std::ios_base::binary | std::ios_base::out );
      frame.write( reinterpret_cast<const char *>( frame_data ), size );
      frame.close();
      break;
    }
  }
  cam->close();
  std::cout << "Camera closed" << std::endl;
  return 0;
}
