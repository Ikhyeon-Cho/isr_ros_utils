cmake_minimum_required(VERSION 3.0.2)
project(isr_ros_utils)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS roscpp)
# find_package(Eigen3 REQUIRED) find_package(PCL 1.10 REQUIRED)

# catkin specific configuration ##
catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS roscpp)

# Build ##
set(CMAKE_BUILD_TYPE Release)
include_directories(include ${catkin_INCLUDE_DIRS})

# Testing ##
if(CATKIN_ENABLE_TESTING)
  find_package(GTest REQUIRED)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

  # # create a gtest executable with target name add_rostest_gtest(
  # tests_isr_ros_utils_core # executable name test/core.test # test launch file
  # test/launch/tester.cpp # test/parameter_test.cpp test/publisher_test.cpp
  # test/subscriber_test.cpp test/timer_test.cpp)
  # target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})

  # create a gtest executable with target name
  add_rostest_gtest(
    test_subscriber_node # executable name
    test/simple_node_example/core.test # test launch file
    test/simple_node_example/test_subscriber_node.cpp # node file
  )
  target_link_libraries(test_subscriber_node ${catkin_LIBRARIES})

endif()
