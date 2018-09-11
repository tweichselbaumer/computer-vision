cmake_minimum_required (VERSION 3.1)
project (computer-vision)

set (CMAKE_CXX_STANDARD 11)

if (MSVC)
     set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /FR /bigobj")
endif (MSVC)

# The version number.
set (Tutorial_VERSION_MAJOR 1)
set (Tutorial_VERSION_MINOR 0)

set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin")
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib")

SET(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" ${PROJECT_SOURCE_DIR}/cmake)

file(GLOB_RECURSE cv_SOURCE_FILES 
	"${PROJECT_SOURCE_DIR}/src/*.h"
	"${PROJECT_SOURCE_DIR}/src/*.cpp"
)

add_definitions(-DWIN32_LEAN_AND_MEAN)
add_definitions(-DLINKUP_BOOST_THREADSAFE)
add_definitions(-DLINKUP_BOOST_CRC16)
add_definitions(-DLINKUP_DEBUG)
add_definitions(-D_USE_MATH_DEFINES)
#add_definitions(-DLINKUP_DEBUG_DETAIL)
add_definitions("-D__SSE3__")

find_package(Eigen3 REQUIRED)

add_subdirectory (${PROJECT_SOURCE_DIR}/thirdParty/LinkUp)
add_subdirectory (${PROJECT_SOURCE_DIR}/thirdParty/dso)


include_directories(
  ${PROJECT_SOURCE_DIR}/thirdParty/dso/src
  ${PROJECT_SOURCE_DIR}/thirdParty/dso/thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
)  

find_package(OpenCV REQUIRED)
message("OpenCV Version: ${OpenCV_VERSION}")
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS system thread filesystem)
include_directories(${Boost_INCLUDE_DIR})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})

FIND_PACKAGE(UEye REQUIRED)
include_directories(${UEye_INCLUDE_DIRS})

message("Files: ${cv_SOURCE_FILES}")

add_executable(computer-vision ${cv_SOURCE_FILES})
target_link_libraries (computer-vision LinkUp)
target_link_libraries (computer-vision dso)
target_link_libraries(computer-vision ${OpenCV_LIBS})
target_link_libraries(computer-vision ${Boost_LIBRARIES})
target_link_libraries(computer-vision ${UEye_LIBRARIES})

IF (WIN32)

ELSE()

add_custom_command(
    TARGET computer-vision 
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
       ${PROJECT_SOURCE_DIR}/bin/computer-vision
        /opt/firefly
)

ENDIF()