cmake_minimum_required(VERSION 3.0.0)

project(tcHarmonicDeformer)   

	set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/modules)
	find_package(Maya REQUIRED)

    #looking for  files
    file(GLOB HEADER_FILES  "*.h")
    file(GLOB SOURCE_FILES "*.cpp")

    include_directories(${CMAKE_CURRENT_SOURCE_DIR} )

    #adding the lib 
    add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${INCLUDES_FILES})
	target_link_libraries(${PROJECT_NAME} PRIVATE Maya::Maya ${MAYA_LOCATION}/lib/tbb.lib)

	#adding maya include headers
	target_include_directories(${PROJECT_NAME} PUBLIC ${HEADER_FILES} Maya::Maya)
	
	target_compile_definitions(${PROJECT_NAME} PUBLIC __TBB_NO_IMPLICIT_LINKAGE=1)
	if(APPLE)
			MESSAGE(STATUS "Using MACOS, not directly supported yet")
	elseif(UNIX)
		set(CXX_DEFINITIONS _BOOL LINUX REQUIRE_IOSTREAM)
		target_compile_definitions(${PROJECT_NAME} PUBLIC ${CXX_DEFINITIONS})
		#making sure to add the common cpp flags, that are defined in the main cpp file
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -pthread -pipe  -Wno-deprecated -fno-gnu-keywords -fPIC")
	elseif(WIN32)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
		set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".mll")
		#copying built lib to the output
	endif()

