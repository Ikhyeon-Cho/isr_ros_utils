#include <ros/ros.h>
#include "isr_ros_utils/core/utils.h"
#include <std_msgs/String.h>

/**
/* Write a simple Timer node.
 */

using namespace isr;

class MockupClass
{
public:
  void timerCallback(const ros::TimerEvent& event)
  {
    ROS_INFO_STREAM("Current Time: " << ros::Time::now() << "\n");
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;

  MockupClass object;

  // [Timer] can be registered in a single line. Call timerCallback in object
  roscpp::Timer timer(ros::Duration(0.1), &MockupClass::timerCallback, &object);
  timer.start();

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}