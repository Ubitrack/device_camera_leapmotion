SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

project(ubitrack_device_camera_leapmotion)

cmake_minimum_required(VERSION 3.4.0)

FIND_PACKAGE(LeapMotion)

include(CheckIncludeFile)

include(${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

include(GNUInstallDirs)
include(cmake/UbitrackConfig)

add_subdirectory(src/leapmotion)

ubitrack_install_utql("doc/utql/*.xml" "doc/utql/*/*.xml"  "doc/utql/*/*/*.xml")
ubitrack_install_doc("doc/utqlDoc/*" "doc/utqlDoc/*/*" "doc/utqlDoc/*/*/*")
