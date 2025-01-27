// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <iostream>
#include <openseekthermal/openseekthermal.hpp>

int main( int, char ** )
{
  auto devices = openseekthermal::listDevices();
  std::cout << "Found " << devices.size() << " devices:" << std::endl;
  for ( const auto &device : devices ) { std::cout << " - " << device << std::endl; }
  return 0;
}
