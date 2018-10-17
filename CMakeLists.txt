cmake_minimum_required (VERSION 3.1)
project (computer-vision)

set (CMAKE_CXX_STANDARD 11)

if (MSVC)
     set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /FR /bigobj /fp:fast /openmp /Ox")
endif (MSVC)

IF (MSVC)
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin")
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib")
else()
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
ENDIF()

SET(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${PROJECT_SOURCE_DIR}/cmake")

file(GLOB_RECURSE cv_SOURCE_FILES 
	"${PROJECT_SOURCE_DIR}/src/*.h"
	"${PROJECT_SOURCE_DIR}/src/*.cpp"
)

set(WITH_DSO 0)
set(WITH_CAMERA 1)
set(WITH_EXTERN_TRIGGER 1)


IF (MSVC)
add_definitions(-DWIN32_LEAN_AND_MEAN)
add_definitions(-D__SSE3__)
add_definitions(-DNOMINMAX)
ENDIF()

add_definitions(-DLINKUP_BOOST_THREADSAFE)
add_definitions(-DLINKUP_BOOST_CRC16)
add_definitions(-DLINKUP_DEBUG)
add_definitions(-D_USE_MATH_DEFINES)
#add_definitions(-DLINKUP_DEBUG_DETAIL)


if (WITH_EXTERN_TRIGGER)
	message("--- extern camera trigger")
	add_definitions("-DEXTERN_CAMERA_TRIGGER")
else ()
	message("--- intern camera trigger")
endif ()

if (WITH_CAMERA)
	message("--- camera enabled")
	add_definitions("-DWITH_CAMERA")
else ()
	message("--- camera disabled")
endif ()
if (WITH_DSO)
find_package(Eigen3 REQUIRED)
message(${EIGEN3_INCLUDE_DIR})
find_package(SuiteParse REQUIRED)
endif ()

add_subdirectory (${PROJECT_SOURCE_DIR}/thirdParty/LinkUp)

find_package(OpenCV REQUIRED)
message("OpenCV Version: ${OpenCV_VERSION}")
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS system thread filesystem)
include_directories(${Boost_INCLUDE_DIR})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})

FIND_PACKAGE(UEye REQUIRED)
include_directories(${UEye_INCLUDE_DIRS})

message("Files: ${cv_SOURCE_FILES}")

find_package(Pangolin 0.2 QUIET)



add_executable(computer-vision ${cv_SOURCE_FILES})

if (WITH_DSO)
	message("--- dso is aktiv")

	add_subdirectory (${PROJECT_SOURCE_DIR}/thirdParty/dso)

	include_directories(
	  ${PROJECT_SOURCE_DIR}/thirdParty/dso/src
	  ${PROJECT_SOURCE_DIR}/thirdParty/dso/thirdparty/Sophus
	  ${EIGEN3_INCLUDE_DIR}
	)  

	add_definitions(-DWITH_DSO)
	if (Pangolin_FOUND)
		message(${Pangolin_INCLUDE_DIRS})
		include_directories( ${Pangolin_INCLUDE_DIRS} ) 
		target_link_libraries (computer-vision dso ${Pangolin_LIBRARIES})
	else()
	target_link_libraries (computer-vision dso )
	endif ()

else ()
	message("--- dso is inaktiv")
endif ()

target_link_libraries (computer-vision LinkUp)
target_link_libraries(computer-vision ${OpenCV_LIBS})
target_link_libraries(computer-vision ${Boost_LIBRARIES} ${BOOST_THREAD_LIBRARY})
target_link_libraries(computer-vision ${UEye_LIBRARIES})


IF (WIN32)
target_link_libraries(computer-vision)
else()

target_link_libraries(computer-vision)

add_custom_command(
    TARGET computer-vision 
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
       ${PROJECT_BINARY_DIR}/${EXECUTABLE_OUTPUT_PATH}/computer-vision
        /opt/firefly
)

ENDIF()