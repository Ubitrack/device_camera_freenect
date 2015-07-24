set(the_description "The UbiTrack FreenectFrameGrabber Component")
ut_add_component(FreenectFrameGrabber DEPS utcore utdataflow utvision )

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# should be extended to recognize V1/V2 and include files accordingly ...
FIND_PACKAGE(Freenect)
IF(FREENECT_FOUND)
	ut_component_include_directories("src/FreenectFrameGrabber" ${UBITRACK_CORE_DEPS_INCLUDE_DIR} ${OPENCV_INCLUDE_DIR} ${TBB_INCLUDE_DIR} ${FREENECT_INCLUDE_DIR})
	ut_glob_component_sources(SOURCES "src/FreenectFrameGrabber/FreenectFrameGrabber.cpp")
	ut_create_single_component(${FREENECT_LIBRARY} ${TBB_ALL_LIBRARIES})
	ut_install_utql_patterns()
ENDIF(FREENECT_FOUND)
