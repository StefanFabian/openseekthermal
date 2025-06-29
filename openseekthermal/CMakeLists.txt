cmake_minimum_required(VERSION 3.8)
project(openseekthermal)
set(CMAKE_CXX_STANDARD 17)

option(BUILD_EXAMPLES "Build examples" ON)
option(DISABLE_LOGGING "Disable all logging" OFF)
option(ENABLE_DEBUG_LOGGING "Enable debug output" OFF)

# find dependencies
find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb REQUIRED libusb-1.0)
if (NOT TARGET PkgConfig::libusb-1.0)
  add_library(PkgConfig::libusb-1.0 INTERFACE IMPORTED)
  target_include_directories(PkgConfig::libusb-1.0 INTERFACE ${libusb_INCLUDE_DIRS})
  target_link_libraries(PkgConfig::libusb-1.0 INTERFACE ${libusb_LIBRARIES})
endif ()

add_library(openseekthermal SHARED
  src/cameras/seek_thermal_camera.cpp
  src/cameras/seek_thermal_compact.cpp
  src/cameras/seek_thermal_compact_pro.cpp
  src/cameras/seek_thermal_nano_300.cpp
  src/usb/seek_device.cpp
  src/exceptions.cpp
  src/frame.cpp
  src/openseekthermal.cpp
)
add_library(openseekthermal::openseekthermal ALIAS openseekthermal)
target_include_directories(openseekthermal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(openseekthermal PRIVATE PkgConfig::libusb-1.0)
# Make compiler complain more
target_compile_options(openseekthermal PRIVATE -Wall -Wextra -Wpedantic)
if (DISABLE_LOGGING)
  target_compile_definitions(openseekthermal PRIVATE DISABLE_LOGGING=1)
elseif (ENABLE_DEBUG_LOGGING)
  target_compile_definitions(openseekthermal PRIVATE ENABLE_DEBUG_LOGGING=1)
endif ()

if (BUILD_EXAMPLES)
  add_executable(grab_raw_frame examples/grab_raw_frame.cpp)
  target_link_libraries(grab_raw_frame openseekthermal)

  add_executable(list_seek_devices examples/list_seek_devices.cpp)
  target_link_libraries(list_seek_devices openseekthermal)

  add_executable(stream_seek_camera examples/stream_seek_camera.cpp)
  target_link_libraries(stream_seek_camera openseekthermal)
endif ()

include(CMakePackageConfigHelpers)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS openseekthermal
  EXPORT openseekthermal-targets
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)
install(
  EXPORT openseekthermal-targets
  FILE openseekthermal-targets.cmake
  NAMESPACE openseekthermal::
  DESTINATION lib/cmake/openseekthermal
)

configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/openseekthermal-config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/openseekthermal-config.cmake"
  INSTALL_DESTINATION lib/cmake/openseekthermal
)

# Generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/openseekthermal-config-version.cmake"
  VERSION 1.0.0
  COMPATIBILITY SameMajorVersion
)

# Install the config files
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/openseekthermal-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/openseekthermal-config-version.cmake"
  DESTINATION lib/cmake/openseekthermal
)

