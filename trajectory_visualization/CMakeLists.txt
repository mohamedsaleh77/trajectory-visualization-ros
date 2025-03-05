cmake_minimum_required(VERSION 3.0.2)
project(trajectory_visualization)

## Compile as C++11 (ROS Kinetic or newer)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  message_generation
)

find_package(jsoncpp REQUIRED)  # Add this line to find jsoncpp

###################################
## Declare ROS messages and srv files
###################################
add_service_files(
  FILES
    SaveTrajectory.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

###################################
## Declare catkin package
###################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    visualization_msgs
    tf2
    tf2_ros
    message_runtime
)

###################################
## Build the node(s)
###################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${jsoncpp_INCLUDE_DIRS}  # Add this line to include jsoncpp headers
)

## Trajectory Publisher and Saver Node
add_executable(trajectory_publisher_saver src/trajectory_publisher_saver.cpp)
add_dependencies(trajectory_publisher_saver
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(trajectory_publisher_saver
  ${catkin_LIBRARIES}
  jsoncpp  # Add this line to link jsoncpp
)

## Trajectory Reader and Publisher Node
add_executable(trajectory_reader_publisher src/trajectory_reader_publisher.cpp)
add_dependencies(trajectory_reader_publisher
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(trajectory_reader_publisher
  ${catkin_LIBRARIES}
  jsoncpp  # Add this line to link jsoncpp
)
