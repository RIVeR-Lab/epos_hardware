cmake_minimum_required(VERSION 2.8.3)
project(epos_library)

find_package(catkin REQUIRED)

# find the 32 or 64 bit libraries
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(ARCH "64bit")
else()
  set(ARCH "32bit")
endif()

## Declare a cpp library
find_library(FTD2XX_LIB NAMES ftd2xx PATHS "lib/${ARCH}")
find_library(EPOSCMD_LIB NAMES EposCmd PATHS "lib/${ARCH}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ftd2xx EposCmd
)

###########
## Build ##
###########

include_directories(include)

add_library(ftd2xx ${FTD2XX_LIB})
add_custom_command(TARGET ftd2xx POST_BUILD COMMAND cp ${FTD2XX_LIB} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/)
set_target_properties(ftd2xx PROPERTIES LINKER_LANGUAGE CXX)

add_library(EposCmd ${EPOSCMD_LIB})
add_custom_command(TARGET EposCmd POST_BUILD COMMAND cp ${EPOSCMD_LIB} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/)
set_target_properties(EposCmd PROPERTIES LINKER_LANGUAGE CXX)


#############
## Install ##
#############

install(TARGETS ftd2xx EposCmd
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
