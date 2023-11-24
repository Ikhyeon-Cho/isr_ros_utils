#include "isr_ros_utils/core/subscriber.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <gtest/gtest.h>

namespace isr
{

class MockCallbackObject
{
public:
  void callback(const std_msgs::StringConstPtr& msg)
  {
    callback_executed = true;
  }

  bool callback_executed{ false };
};

class SubscriberTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Register to ROS Parameter Server
    ros::param::set("topic_key", "topic_from_server");  // Saved as "/topic_from_server"

    // Publish msg to trigger callback
    pub = nh.advertise<std_msgs::String>("topic", 10);
  }

  ros::NodeHandle nh;
  ros::NodeHandle nh_private{ "test" };
  ros::Publisher pub;

  MockCallbackObject object;
  using StringSubscriber = roscpp::Subscriber<std_msgs::String>;
};

TEST_F(SubscriberTest, constructors)
{
  // Constructor with topic
  StringSubscriber subscriber_1("topic_normal", &MockCallbackObject::callback, &object);
  EXPECT_STREQ(subscriber_1.getTopic().c_str(), "/topic_normal");  // Subscriber has global namespace as default
  EXPECT_EQ(subscriber_1.getQueueSize(), 10);

  // Constructor with topic and queue size
  StringSubscriber subscriber_2("topic_normal", 5, &MockCallbackObject::callback, &object);
  EXPECT_STREQ(subscriber_2.getTopic().c_str(), "/topic_normal");
  EXPECT_EQ(subscriber_2.getQueueSize(), 5);

  // Constructor with nodeHandle, topic
  StringSubscriber subscriber_3{ nh_private, "topic_normal", &MockCallbackObject::callback, &object };
  EXPECT_STREQ(subscriber_3.getTopic().c_str(), "/test/topic_normal");

  // Constructor with ros parameter (topic)
  roscpp::Parameter<std::string> topic_param{ "topic_key", "topic_default" };
  StringSubscriber subscriber_4{ topic_param, &MockCallbackObject::callback, &object };
  EXPECT_STREQ(subscriber_4.getTopic().c_str(), "/topic_from_server");
}

TEST_F(SubscriberTest, subscriberCallback)
{
  StringSubscriber subscriber("topic", &MockCallbackObject::callback, &object);
  EXPECT_FALSE(object.callback_executed);

  ros::Rate rate(100);
  while (!object.callback_executed && ros::ok())
  {
    pub.publish(std_msgs::String());
    ros::spinOnce();
    rate.sleep();
  }

  EXPECT_TRUE(object.callback_executed);
}

}  // namespace isr