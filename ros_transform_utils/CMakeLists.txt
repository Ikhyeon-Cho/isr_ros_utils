cmake_minimum_required(VERSION 3.0.2)
project(ros_transform_utils)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS 
  tf2_ros 
  tf2_eigen 
  tf2_geometry_msgs
)

# catkin specific configuration ##
catkin_package(
  INCLUDE_DIRS 
    include 
  LIBRARIES 
    ${PROJECT_NAME} 
  CATKIN_DEPENDS 
    tf2_ros tf2_eigen tf2_geometry_msgs
)

# Build ##
set(CMAKE_BUILD_TYPE Release)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
)

# Testing ##
if(CATKIN_ENABLE_TESTING)
  find_package(GTest REQUIRED)
  find_package(rostest REQUIRED) # TEST with roscore
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

endif()