# ===================================================================================
#  DBoW3 CMake configuration file
#
#             ** File generated automatically, do not modify **
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    FIND_PACKAGE(DBoW3 REQUIRED )
#    TARGET_LINK_LIBRARIES(MY_TARGET_NAME )
#
#    This file will define the following variables:
#      - DBoW3_LIBS          : The list of libraries to links against.
#      - DBoW3_LIB_DIR       : The directory where lib files are. Calling LINK_DIRECTORIES
#                                with this path is NOT needed.
#      - DBoW3_VERSION       : The  version of this PROJECT_NAME build. Example: "1.2.0"
#      - DBoW3_VERSION_MAJOR : Major version part of VERSION. Example: "1"
#      - DBoW3_VERSION_MINOR : Minor version part of VERSION. Example: "2"
#      - DBoW3_VERSION_PATCH : Patch version part of VERSION. Example: "0"
#
# ===================================================================================
SET(DBoW3_INCLUDE_DIRS "C:/SDKs/DBo3W/include")

SET(DBoW3_LIB_DIR "C:/SDKs/DBo3W/lib")

SET(DBoW3_LIBS "${DBoW3_LIB_DIR}/DBoW3001.lib")
SET(DBoW3_LIBRARIES "${DBoW3_LIB_DIR}/DBoW3001.lib")

SET(DBoW3_FOUND YES)
SET(DBoW3_FOUND "YES")
SET(DBoW3_VERSION        0.0.1)
SET(DBoW3_VERSION_MAJOR  0)
SET(DBoW3_VERSION_MINOR  0)
SET(DBoW3_VERSION_PATCH  1)
