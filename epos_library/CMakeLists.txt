cmake_minimum_required(VERSION 2.8.3)
project(epos_library)

find_package(catkin REQUIRED)

execute_process(
  COMMAND ./download_library.sh
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/
)

# find the 32 or 64 bit libraries
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(ARCH "x86_64")
else()
  set(ARCH "x86")
endif()
message(STATUS "Detected architecture: ${ARCH}")

catkin_package(
  LIBRARIES ftd2xx EposCmd
)

###########
## Build ##
###########

# Copy Library definitions header to devel space
file(COPY "EPOS_Linux_Library/include/Definitions.h" DESTINATION "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}")

add_library(ftd2xx ${FTD2XX_LIB})
add_custom_command(TARGET ftd2xx POST_BUILD COMMAND cp ${PROJECT_SOURCE_DIR}/EPOS_Linux_Library/lib/${ARCH}/libftd2xx.so.1.1.12 ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libftd2xx.so)
set_target_properties(ftd2xx PROPERTIES LINKER_LANGUAGE CXX)

add_library(EposCmd ${EPOSCMD_LIB})
add_custom_command(TARGET EposCmd POST_BUILD COMMAND cp ${PROJECT_SOURCE_DIR}/EPOS_Linux_Library/lib/${ARCH}/libEposCmd.so.5.0.1.0 ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libEposCmd.so)
set_target_properties(EposCmd PROPERTIES LINKER_LANGUAGE CXX)

#############
## Install ##
#############

install(TARGETS ftd2xx EposCmd
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/Definitions.h"
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
