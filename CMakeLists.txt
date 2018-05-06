cmake_minimum_required (VERSION 3.1)
project (computer-vision)

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
add_subdirectory (${PROJECT_SOURCE_DIR}/thirdParty/MathFunctions)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(computer-vision ${cv_SOURCE_FILES})
target_link_libraries (computer-vision MathFunctions)
target_link_libraries(computer-vision ${OpenCV_LIBS})