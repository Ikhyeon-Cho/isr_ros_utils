#include <ros/ros.h>
#include <gtest/gtest.h>

#include "isr_ros_utils/core/utils.h"
#include <std_msgs/String.h>

namespace isr
{

class PublisherTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Register to ROS Parameter Server
    ros::param::set("topic", "topic_from_server");  // Saved as "/test_node/topic_from_server"
  }

  roscpp::Publisher<std_msgs::String> local_topic_publisher_{ "topic" };
  roscpp::Publisher<std_msgs::String> global_topic_publisher_{ "/topic", 5 };
  ros::NodeHandle nodeHandle_{ "new_namespace" };
  roscpp::Publisher<std_msgs::String> new_namespace_topic_publisher_{ nodeHandle_, "topic" };
};

class TestSubscriberNode
{
public:
  TestSubscriberNode();
  void stringCallback(const std_msgs::String::ConstPtr& msg);
  void timerCallback(const ros::TimerEvent& event);

private:
  // Parameters
  roscpp::Parameter<std::string> sub_topic_param_{ "", "" };

  roscpp::Publisher<std_msgs::String> publisher_{ "/test" };
  roscpp::Subscriber<std_msgs::String> subscriber_{ "/test", &TestSubscriberNode::stringCallback, this };
  roscpp::Timer timer_{ ros::Duration(0.1), &TestSubscriberNode::timerCallback, this, true };
};

TestSubscriberNode::TestSubscriberNode()
{
}

void TestSubscriberNode::stringCallback(const std_msgs::String::ConstPtr& msg)
{
  std::cout << "test" << std::endl;
}

TEST(a, b)
{
  EXPECT_TRUE(true);
}

void TestSubscriberNode::timerCallback(const ros::TimerEvent& event)
{
  std::stringstream ss;
  ss << "test";

  std_msgs::String msg;
  msg.data = ss.str();
  ROS_INFO("%s", msg.data.c_str());
  publisher_.publish(msg);
}

}  // namespace isr

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_node_example");
  ros::NodeHandle nh;

  isr::TestSubscriberNode node;

  int count = 0;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}