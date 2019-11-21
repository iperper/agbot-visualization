cmake_minimum_required(VERSION 2.8.3)
project(visualization)

add_compile_options(-std=c++14)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  gflags_catkin
  glog_catkin
  sensor_msgs
  message_filters
  message_generation
)

add_message_files(
  FILES
  StampedInt.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES visualization
  CATKIN_DEPENDS message_runtime
                geometry_msgs 
                roscpp
                std_msgs
                gflags_catkin
                glog_catkin
                sensor_msgs
                message_filters
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/visualization_node.cpp)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})