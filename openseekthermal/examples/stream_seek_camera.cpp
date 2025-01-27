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

  using clock = std::chrono::high_resolution_clock;
  auto start = clock::now();
  const int FRAME_COUNT = 2000;
  std::ofstream file( "headers.csv" );
  for ( int i = 0; i < FRAME_COUNT; ++i ) {
    FrameHeader header;
    size_t size;
    GrabFrameResult result = cam->grabFrame( nullptr, size, &header );
    if ( result != GrabFrameResult::SUCCESS )
      continue;
    std::cout << "Frame " << std::dec << header.getFrameNumber() << ", Type "
              << static_cast<int>( header._getRawFrameType() ) << " ("
              << to_string( header.getFrameType() ) << ")" << std::endl;
    const auto *data = reinterpret_cast<const uint16_t *>( header.data().data() );
    size_t header_size = header.data().size() / 2;
    for ( size_t i = 0; i < header_size; ++i ) {
      uint16_t value = le16toh( data[i] );
      file << static_cast<int>( value ) << ",";
      std::cout << std::hex << std::setfill( '0' ) << std::setw( 4 ) << static_cast<int>( value )
                << " ";
    }
    file.seekp( -1, std::ios_base::cur );
    file << std::endl;
    std::cout << std::endl << "----------------" << std::endl;
  }
  file.close();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>( clock::now() - start ).count();
  std::cout << "Framerate: " << std::setprecision( 3 ) << ( FRAME_COUNT * 1000.0 / ms ) << " fps"
            << std::endl;
  cam->close();
  std::cout << "Camera closed" << std::endl;
  return 0;
}
