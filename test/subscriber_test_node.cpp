#include <ros/ros.h>
#include "isr_ros_utils/core/utils.h"
#include <std_msgs/String.h>

/**
/* Write a simple subscriber node. Topic names are loaded from ROS Parameter Sever
 */

using namespace isr;

class Listener
{
public:
  void subscriberCallback(const std_msgs::String::ConstPtr& msg)
  {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;

  Listener listener;

  roscpp::Parameter<std::string> topic_parameter("Subscriber/Topic/StringSubscriber", "topic_default_when_no_param");

  // [Subscriber] can be initialized in a single line
  roscpp::Subscriber<std_msgs::String> string_subscriber(topic_parameter, &Listener::subscriberCallback, &listener);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();  // check callback queues
    loop_rate.sleep();
  }
}