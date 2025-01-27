cmake_minimum_required(VERSION 3.8)
project(openseekthermal_gstreamer)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake QUIET)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-plugins-base-1.0 gstreamer-pbutils-1.0)
find_package(openseekthermal REQUIRED)

add_library(openseekthermal_gstreamer SHARED
  src/gstopenseekthermalplugin.cpp
  src/gstopenseekthermalsrc.cpp
)
target_include_directories(openseekthermal_gstreamer PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(openseekthermal_gstreamer PUBLIC openseekthermal::openseekthermal ${GST_LIBRARIES})
target_include_directories(openseekthermal_gstreamer PRIVATE ${GST_INCLUDE_DIRS})

include(GNUInstallDirs)
install(TARGETS openseekthermal_gstreamer
  EXPORT openseekthermal_gstreamer-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/openseekthermal_gstreamer
)

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_clang_format(CONFIG_FILE "${CMAKE_SOURCE_DIR}/../.clang-format")
  ament_lint_auto_find_test_dependencies()
endif()

if (ament_cmake_FOUND)
  ament_environment_hooks(register_plugin.dsv.in)
  ament_package()
endif()
