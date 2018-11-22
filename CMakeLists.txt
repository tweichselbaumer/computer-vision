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

add_definitions(-DIMAGE_WIDTH=512)
add_definitions(-DIMAGE_HEIGHT=512)

set(WITH_DSO 1)
set(WITH_CAMERA 0)
set(WITH_EXTERN_TRIGGER 0)


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
	find_package(OpenCV REQUIRED)
	find_package(Glog REQUIRED)
	find_package(Pangolin REQUIRED)
	find_package(G2O REQUIRED)
	find_package(DBoW3 REQUIRED)
	find_package(Boost REQUIRED COMPONENTS system)

	include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIR}
        ${GLOG_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
		${G2O_INCLUDE_DIR}
		${DBoW3_INCLUDE_DIRS}
		${Boost_INCLUDE_DIR}

        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
	)

	if (MSVC)
		add_definitions("-D__SSE3__")
		add_definitions("-DNOMINMAX")
		add_definitions("-D_USE_MATH_DEFINES")
		add_definitions("-DGOOGLE_GLOG_DLL_DECL=")
		add_definitions("-DGLOG_NO_ABBREVIATED_SEVERITIES=")
		add_definitions("-DWIN32_LEAN_AND_MEAN=")
	else()
		add_definitions("-DENABLE_SSE")
		set(CMAKE_CXX_FLAGS "-Wall -Wno-deprecated -march=native -Wno-duplicate-decl-specifier -Wno-ignored-qualifiers -Wno-reorder -pthread")
	endif (MSVC)

endif ()

add_subdirectory (${PROJECT_SOURCE_DIR}/thirdParty/LinkUp)

find_package(OpenCV REQUIRED)
message("OpenCV Version: ${OpenCV_VERSION}")
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS system thread filesystem)
include_directories(${Boost_INCLUDE_DIR})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})

if (WITH_CAMERA)
	FIND_PACKAGE(UEye REQUIRED)
	include_directories(${UEye_INCLUDE_DIRS})
endif ()

message("Files: ${cv_SOURCE_FILES}")

add_executable(computer-vision ${cv_SOURCE_FILES})

if (WITH_DSO)
	message("--- dso is aktiv")

	add_subdirectory (${PROJECT_SOURCE_DIR}/thirdParty/LDSO)

	include_directories(
	  ${PROJECT_SOURCE_DIR}/thirdParty/LDSO/include
	  ${PROJECT_SOURCE_DIR}/thirdParty/LDSO/thirdparty/Sophus
	  ${EIGEN3_INCLUDE_DIR}
	)  

	add_definitions(-DWITH_DSO)
	if (Pangolin_FOUND)
		message(${Pangolin_INCLUDE_DIRS})
		include_directories( ${Pangolin_INCLUDE_DIRS} ) 
		target_link_libraries (computer-vision ldso ${Pangolin_LIBRARIES})
	else()
	target_link_libraries (computer-vision ldso )
	endif ()

else ()
	message("--- dso is inaktiv")
endif ()

target_link_libraries (computer-vision LinkUp)
target_link_libraries(computer-vision ${OpenCV_LIBS})
target_link_libraries(computer-vision ${Boost_LIBRARIES} ${BOOST_THREAD_LIBRARY})

if (WITH_CAMERA)
	target_link_libraries(computer-vision ${UEye_LIBRARIES})
endif ()

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