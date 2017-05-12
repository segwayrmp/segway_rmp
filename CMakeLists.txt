cmake_minimum_required(VERSION 2.8.3)
project(segway_rmp)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    message_generation
    nav_msgs
    roscpp
    serial
    std_msgs
    tf
)

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(libsegwayrmp REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES SegwayStatus.msg SegwayStatusStamped.msg
)

generate_messages(DEPENDENCIES std_msgs)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${libsegwayrmp_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  DEPENDS libsegwayrmp
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    nav_msgs
    roscpp
    serial
    std_msgs
    tf
)

add_executable(segway_rmp_node src/segway_rmp_node.cpp)

message("libsegwayrmp_LIBRARIES: ${libsegwayrmp_LIBRARIES}")

add_dependencies(segway_rmp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(segway_rmp_node
  ${Boost_LIBRARIES}
  ${libsegwayrmp_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS segway_rmp_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
