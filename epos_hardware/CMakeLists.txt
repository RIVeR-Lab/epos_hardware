cmake_minimum_required(VERSION 2.8.3)
project(epos_hardware)

find_package(catkin REQUIRED COMPONENTS
  epos_library
  hardware_interface
  transmission_interface
  controller_manager
  roscpp
  diagnostic_updater
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES epos_library_utils epos_manager epos_hardware
  CATKIN_DEPENDS epos_library hardware_interface roscpp controller_manager diagnostic_updater
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# A collection of utilities for using the EPOS Command Libary
add_library(epos_library_utils
  src/util/utils.cpp
)
target_link_libraries(epos_library_utils
  ${catkin_LIBRARIES}
)


# Build tool to list available devices
add_executable(list_devices src/tools/list_devices.cpp)
target_link_libraries(list_devices
  ${catkin_LIBRARIES}
  epos_library_utils
)

# Build tool to list a devices state
add_executable(get_state src/tools/get_state.cpp)
target_link_libraries(get_state
  ${catkin_LIBRARIES}
  epos_library_utils
)

add_library(epos_manager
  src/util/epos.cpp src/util/epos_manager.cpp
)
target_link_libraries(epos_manager
  ${catkin_LIBRARIES}
  epos_library_utils
)

add_library(epos_hardware
  src/util/epos_hardware.cpp
)
target_link_libraries(epos_hardware
  ${catkin_LIBRARIES}
  epos_manager
  epos_library_utils
)


add_executable(epos_hardware_node src/nodes/epos_hardware_node.cpp)
target_link_libraries(epos_hardware_node
  ${catkin_LIBRARIES}
  epos_hardware
)

#############
## Install ##
#############


# Mark udev install program for installation
install(PROGRAMS
  install_udev_rules
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES 90-ftd2xx.rules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark libraries and nodes for installation
install(TARGETS epos_library_utils epos_manager epos_hardware list_devices get_state epos_hardware_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
