cmake_minimum_required(VERSION 2.8.3)
project(segway_rmp)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp tf message_generation geometry_msgs nav_msgs serial)

# libsegwyarmp provides segwayrmp and ftd2xx
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSEGWAYRMP REQUIRED libsegwayrmp)
find_library(LIBSEGWAYRMP_LIBRARY
  NAMES segwayrmp
  PATHS ${LIBSEGWAYRMP_LIBRARY_DIRS}
)
find_library(LIBFTD2XX_LIBRARY
  NAMES ftd2xx
  PATHS ${LIBSEGWAYRMP_LIBRARY_DIRS}
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
add_message_files(
  DIRECTORY msg
  FILES SegwayStatus.msg SegwayStatusStamped.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs
)

include_directories(${Boost_INCLUDE_DIRS} ${LIBSEGWAYRMP_INCLUDE_DIRS})

add_executable(segway_rmp_node src/segway_rmp_node.cpp)
find_package(Boost REQUIRED COMPONENTS system thread)

target_link_libraries(
  segway_rmp_node 
  ${Boost_LIBRARIES} 
  ${LIBSEGWAYRMP_LIBRARY} 
  ${LIBFTD2XX_LIBRARY} 
  ${catkin_LIBRARIES}
)

## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS libsegwayrmp
    CATKIN_DEPENDS roscpp tf geometry_msgs nav_msgs std_msgs serial
)
