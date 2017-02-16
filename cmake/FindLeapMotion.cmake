# This script locates the PERC SDK
# ------------------------------------
#
# usage:
# find_package(PERC ...)
#
# searches in LEAP_ROOT and usual locations
#
# Sets LEAP_INCLUDE_DIR, LEAP_LIBRARY

SET( LEAP_LIB_SUBPATH "x86" )
IF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  SET( LEAP_LIB_SUBPATH "x64" )
ENDIF( CMAKE_SIZEOF_VOID_P EQUAL 8 )


set(LEAP_POSSIBLE_PATHS
    ${LEAP_SDK}
    $ENV{LEAP_SDK}
)

find_path(LEAP_INCLUDE_DIR 
    NAMES Leap.h
    PATH_SUFFIXES "include"
    PATHS ${LEAP_POSSIBLE_PATHS}
)

find_library(LEAP_LIBRARY
    NAMES Leap Leapd
    PATH_SUFFIXES "lib/${LEAP_LIB_SUBPATH}"
    PATHS ${LEAP_POSSIBLE_PATHS}
)

find_package_handle_standard_args(LEAP DEFAULT_MSG LEAP_LIBRARY LEAP_INCLUDE_DIR)

