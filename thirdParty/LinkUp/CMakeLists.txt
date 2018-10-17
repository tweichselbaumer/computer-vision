set(linkup_SOURCE_FILES
  "${PROJECT_SOURCE_DIR}/thirdParty/LinkUp/src/CRC16.cpp"
  "${PROJECT_SOURCE_DIR}/thirdParty/LinkUp/src/CRC16.h"
  "${PROJECT_SOURCE_DIR}/thirdParty/LinkUp/src/LinkUpNode.cpp"
  "${PROJECT_SOURCE_DIR}/thirdParty/LinkUp/src/LinkUpNode.h"
  "${PROJECT_SOURCE_DIR}/thirdParty/LinkUp/src/LinkUpRaw.cpp"
  "${PROJECT_SOURCE_DIR}/thirdParty/LinkUp/src/LinkUpRaw.h"
  "${PROJECT_SOURCE_DIR}/thirdParty/LinkUp/src/Platform.h"
  "${PROJECT_SOURCE_DIR}/thirdParty/LinkUp/src/Platform.cpp"
  "${PROJECT_SOURCE_DIR}/thirdParty/LinkUp/src/AvlTree.h"
  "${PROJECT_SOURCE_DIR}/thirdParty/LinkUp/src/AvlTree.cpp"
  "${PROJECT_SOURCE_DIR}/thirdParty/LinkUp/src/LinkUpLabel.cpp"
  "${PROJECT_SOURCE_DIR}/thirdParty/LinkUp/src/LinkUpLabel.h"
  "${PROJECT_SOURCE_DIR}/thirdParty/LinkUp/src/LinkUpLogic.h"
  "${PROJECT_SOURCE_DIR}/thirdParty/LinkUp/src/LinkedList.h"
  "${PROJECT_SOURCE_DIR}/thirdParty/LinkUp/src/LinkedList.cpp"
  "${PROJECT_SOURCE_DIR}/thirdParty/LinkUp/src/LinkUpPropertyLabel.h"
  "${PROJECT_SOURCE_DIR}/thirdParty/LinkUp/src/LinkUpPropertyLabel.cpp"
  "${PROJECT_SOURCE_DIR}/thirdParty/LinkUp/src/LinkUpEventLabel.h"
  "${PROJECT_SOURCE_DIR}/thirdParty/LinkUp/src/LinkUpEventLabel.cpp"
  "${PROJECT_SOURCE_DIR}/thirdParty/LinkUp/src/LinkUpFunctionLabel.cpp"
  "${PROJECT_SOURCE_DIR}/thirdParty/LinkUp/src/LinkUpFunctionLabel.h"
)

add_library(LinkUp ${linkup_SOURCE_FILES})
target_include_directories(LinkUp PUBLIC "src")

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIR})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
