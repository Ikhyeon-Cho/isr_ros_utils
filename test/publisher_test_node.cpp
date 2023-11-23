#include <ros/ros.h>
#include "isr_ros_utils/core/utils.h"
#include <std_msgs/String.h>

/**
/* Write a simple publisher node. Topic names are loaded from ROS Parameter Sever
 */

using namespace isr;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;

  roscpp::Parameter<std::string> topic_parameter("Publisher/Topic/StringPublisher", "topic_default_when_no_param");

  // [Publisher] can be registered in a single line
  roscpp::Publisher<std_msgs::String> string_publisher(topic_parameter, 10);

  int count = 0;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std::stringstream ss;
    ss << "hello world " << count;

    std_msgs::String msg;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());

    string_publisher.publish(msg);
    ++count;

    loop_rate.sleep();
  }
}