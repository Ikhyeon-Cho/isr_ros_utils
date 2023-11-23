cmake_minimum_required(VERSION 3.0.2)
project(isr_ros_utils)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.10 REQUIRED)


###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS 
    include
 CATKIN_DEPENDS 
    roscpp 
)



###########
## Build ##
###########

set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(GTest REQUIRED)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

  # catkin_add_gtest(${PROJECT_NAME}-test
  #     test/tester.cpp
  #     test/parameter_test.cpp
  # )

  add_rostest_gtest(${PROJECT_NAME}-test
      test/launch/test.launch
      test/launch/tester.cpp
      test/parameter_test.cpp
      test/publisher_test.cpp
      test/subscriber_test.cpp
      test/timer_test.cpp
  )

  target_link_libraries(${PROJECT_NAME}-test
      ${catkin_LIBRARIES}
  )

  add_executable(publisher_test_node 
    test/publisher_test_node.cpp
  )
  target_link_libraries(publisher_test_node 
    ${catkin_LIBRARIES}
  )

  add_executable(subscriber_test_node 
    test/subscriber_test_node.cpp
  )
  target_link_libraries(subscriber_test_node 
    ${catkin_LIBRARIES}
  )

  add_executable(timer_test_node 
    test/timer_test_node.cpp
  )
  target_link_libraries(timer_test_node 
    ${catkin_LIBRARIES}
  )

endif()