set(the_description "The UbiTrack LeapMotion Component")
ut_add_component(leapmotion DEPS utcore utdataflow utvision )

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

FIND_PACKAGE(LeapMotion)

IF(LEAP_FOUND)
  ut_component_include_directories("src/leapmotion" ${LEAP_INCLUDE_DIR} ${UBITRACK_CORE_DEPS_INCLUDE_DIR})
  ut_glob_component_sources(HEADERS "src/leapmotion/*.h" SOURCES "src/leapmotion/*.cpp")
  ut_create_single_component(${LEAP_LIBRARY})
ENDIF(LEAP_FOUND)

ut_install_utql_patterns()
