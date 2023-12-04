cmake_minimum_required(VERSION 3.0.2)
project(isr_ros_utils)
add_compile_options(-std=c++14)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Eigen3 REQUIRED) 

# catkin specific configuration ##
catkin_package(INCLUDE_DIRS include LIBRARIES isr_ros_utils CATKIN_DEPENDS roscpp)

# Build ##
set(CMAKE_BUILD_TYPE Release)
include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# Declare a C++ library ##
add_library(${PROJECT_NAME}
  src/MsgConverter.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Testing ##
if(CATKIN_ENABLE_TESTING)
  find_package(GTest REQUIRED)
  find_package(rostest REQUIRED) # TEST with roscore
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

  # Test Parameter 
  add_rostest_gtest(
    test_parameter # executable name
    test/Parameter/parameter.test # test launch file
    test/Parameter/parameter_test.cpp # node file
  )
  target_link_libraries(test_parameter ${catkin_LIBRARIES})

  # Test Publisher
  add_rostest_gtest(
    test_publisher # executable name
    test/Publisher/publisher.test # test launch file
    test/Publisher/publisher_test.cpp # node file
  )
  target_link_libraries(test_publisher ${catkin_LIBRARIES})

  # Test Subscriber
  add_rostest_gtest(
    test_subscriber # test executable name
    test/Subscriber/subscriber.test # test launch file
    test/Subscriber/subscriber_test.cpp # test file
  )
  target_link_libraries(test_subscriber ${catkin_LIBRARIES})

  # Test Timer
  add_rostest_gtest(
    test_timer # test executable name
    test/Timer/timer.test # test launch file
    test/Timer/timer_test.cpp # test file
  )
  target_link_libraries(test_timer ${catkin_LIBRARIES})

endif()
