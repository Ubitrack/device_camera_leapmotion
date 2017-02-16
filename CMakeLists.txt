IF(WIN32)
	set(the_description "The UbiTrack Leap Motion Component")
	ut_add_component(LeapMotionDriver DEPS utcore utdataflow)
	
	SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
	MESSAGE(WARN " TEST1 ------------------")
	

	FIND_PACKAGE(Leap)
	

	
	
	MESSAGE(WARN " TEST2 ------------------")

	#IF(Leap_FOUND)
		ut_component_include_directories("src/leapmotion" ${TINYXML_INCLUDE_DIR} ${LOG4CPP_INCLUDE_DIR} ${BOOSTBINDINGS_INCLUDE_DIR} ${LAPACK_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${LEAP_INCLUDE_DIR})
		ut_glob_component_sources(HEADERS "src/leapmotion/*.h" SOURCES "src/leapmotion/*.cpp")
		ut_create_single_component(${LEAP_LIBRARY})
		ut_install_utql_patterns()

	#ELSE(Leap_FOUND)
	#	MESSAGE(WARN " Leap Motions was not found - disable leap motion driver")
	#	SET(BUILD_LeapMotionDriver OFF CACHE BOOL "Build Leap Motion deactivated" FORCE)
	#ENDIF(Leap_FOUND)

	MESSAGE(WARN " TEST3 ------------------")
ENDIF(WIN32)