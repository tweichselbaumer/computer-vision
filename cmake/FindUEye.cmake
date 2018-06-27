# Find the native UEye headers and libraries.
#
#  UEye_INCLUDE_DIRS - where to find uEye.h, etc.
#  UEye_LIBRARIES    - List of libraries when using UEye.
#  UEye_FOUND        - True if UEye found.

SET(UEye_ROOT_DIR
    "$ENV{UEye_ROOT_DIR}")

FIND_PATH(UEye_INCLUDE_DIRS NAMES uEye.h HINTS ${UEye_ROOT_DIR}/include )

IF(CMAKE_SIZEOF_VOID_P MATCHES "8")
	FIND_LIBRARY(UEye_LIBRARY NAMES uEye_api_64 ueye_tools_64 HINTS ${UEye_ROOT_DIR} ${UEye_ROOT_DIR}/Lib)
	FIND_LIBRARY(UEye_LIBRARY_DEBUG NAMES uEye_api_64 ueye_tools_64 HINTS ${UEye_ROOT_DIR} ${UEye_ROOT_DIR}/Lib)
ELSE()
    # Look for the library.
	FIND_LIBRARY(UEye_LIBRARY NAMES uEye_api uEye_tools HINTS ${UEye_ROOT_DIR} ${UEye_ROOT_DIR}/Lib)
	FIND_LIBRARY(UEye_LIBRARY_DEBUG NAMES uEye_api uEye_tools HINTS ${UEye_ROOT_DIR} ${UEye_ROOT_DIR}/Lib)
ENDIF()

MARK_AS_ADVANCED(UEye_LIBRARY)
MARK_AS_ADVANCED(UEye_LIBRARY_DEBUG)


SET(UEye_LIBRARIES optimized ${UEye_LIBRARY} debug ${UEye_LIBRARY_DEBUG})

#Message("rest" $ENV{UEye_ROOT_DIR})
#Message("rest" ${UEye_ROOT_DIR})
#Message("rest" ${UEye_LIBRARIES})
#Message("rest" ${UEye_INCLUDE_DIRS})

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(UEye DEFAULT_MSG UEye_LIBRARIES UEye_INCLUDE_DIRS)

MARK_AS_ADVANCED(UEye_LIBRARIES UEye_INCLUDE_DIRS)
