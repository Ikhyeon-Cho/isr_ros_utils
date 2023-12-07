cmake_minimum_required(VERSION 3.0.2)
project(ros_pcl_utils)
add_compile_options(-std=c++14)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  ros_transform_utils
)

# catkin specific configuration ##
catkin_package(
  INCLUDE_DIRS 
    include 
  LIBRARIES 
    ${PROJECT_NAME} 
  CATKIN_DEPENDS 
    pcl_conversions
    ros_transform_utils

)

# Build ##
set(CMAKE_BUILD_TYPE Release)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

# Declare a C++ library ##
add_library(${PROJECT_NAME}
  src/PointcloudProcessor.cpp
)


# Testing ##
if(CATKIN_ENABLE_TESTING)
  find_package(GTest REQUIRED)
  find_package(rostest REQUIRED) # TEST with roscore
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

endif()
