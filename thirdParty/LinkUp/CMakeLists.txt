set(linkup_SOURCE_FILES
  "src/CRC16.cpp"
  "src/CRC16.h"
  "src/LinkUpNode.cpp"
  "src/LinkUpNode.h"
  "src/LinkUpRaw.cpp"
  "src/LinkUpRaw.h"
  "src/Platform.h"
  "src/Platform.cpp"
  "src/AvlTree.h"
  "src/AvlTree.cpp"
  "src/LinkUpLabel.cpp"
  "src/LinkUpLabel.h"
  "src/LinkUpLogic.h"
  "src/LinkedList.h"
  "src/LinkedList.cpp"
  "src/LinkUpPropertyLabel.h"
  "src/LinkUpPropertyLabel.cpp"
  "src/LinkUpEventLabel.h"
  "src/LinkUpEventLabel.cpp"
)

add_library(LinkUp ${linkup_SOURCE_FILES})
target_include_directories(LinkUp PUBLIC "src")

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIR})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})