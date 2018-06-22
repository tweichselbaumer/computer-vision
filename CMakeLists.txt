cmake_minimum_required (VERSION 3.1)
project (computer-vision)

set (CMAKE_CXX_STANDARD 11)

# The version number.
set (Tutorial_VERSION_MAJOR 1)
set (Tutorial_VERSION_MINOR 0)

set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin")
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

file(GLOB_RECURSE cv_SOURCE_FILES "${PROJECT_SOURCE_DIR}/src"
    "*.h"
    "*.cpp"
)

add_definitions(-DLINKUP_BOOST_THREADSAFE)
add_definitions(-DLINKUP_BOOST_CRC16)

add_subdirectory (${PROJECT_SOURCE_DIR}/thirdParty/LinkUp)

find_package(OpenCV REQUIRED)
message("OpenCV Version: "${OpenCV_VERSION})
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIR})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})

add_executable(computer-vision ${cv_SOURCE_FILES})
target_link_libraries (computer-vision LinkUp)
target_link_libraries(computer-vision ${OpenCV_LIBS})
target_link_libraries(computer-vision ${Boost_LIBRARIES})
