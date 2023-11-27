#include "isr_ros_utils/core/Subscriber.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <gtest/gtest.h>

class SubscriberTest : public ::testing::Test
{
protected:
  using StringSub = roscpp::Subscriber<std_msgs::String>;

  void SetUp() override
  {
    // Register to ROS Parameter Server
    ros::param::set("topic_key", "topic_from_param_server");
  }

  void callback(const std_msgs::StringConstPtr& msg)
  {
    callback_executed = true;
  }

  ros::NodeHandle nh;
  ros::NodeHandle nh_private{ "test" };

  // 1. Subscriber initialized with topic name. When subscribed, callback is triggered
  StringSub subscriber_1_{ "topic_1", &SubscriberTest::callback, this };

  // 2. We can also specify queue size within constructor
  StringSub subscriber_2_{ "topic_2", 5, &SubscriberTest::callback, this };

  // 3. We can also specify namespace (prefix) of the topic name with constructor
  StringSub subscriber_3_{ nh_private, "topic_3", &SubscriberTest::callback, this };

  // 4. Subscriber can use roscpp::Parameter to initialize with parameterized topic name
  roscpp::Parameter<std::string> topic_param{ "topic_key", "topic_4" };  // read {topic_key} from parameter server.
                                                                         // If fails, use "topic_4"
  StringSub subscriber_4_{ topic_param, &SubscriberTest::callback, this };

  bool callback_executed{ false };
};

TEST_F(SubscriberTest, constructors)
{
  EXPECT_STREQ(subscriber_1_.getTopic().c_str(), "/topic_1");  // Subscriber has global namespace as default
  EXPECT_EQ(subscriber_1_.getQueueSize(), 10);                 // Default queue size is expected to be 10

  EXPECT_STREQ(subscriber_2_.getTopic().c_str(), "/topic_2");
  EXPECT_EQ(subscriber_2_.getQueueSize(), 5);  // Obviously, queue size is expected to be 5

  EXPECT_STREQ(subscriber_3_.getTopic().c_str(),
               "/test/topic_3");  // Now, subscriber topic has nh_private's namespace

  EXPECT_STREQ(subscriber_4_.getTopic().c_str(),
               "/topic_from_param_server");  // Now subscribes to parameterized topic name
}

TEST_F(SubscriberTest, Callback)
{
  EXPECT_FALSE(callback_executed);  // At first, callback has not been executed

  ros::Publisher publisher = nh.advertise<std_msgs::String>("topic_1", 10);

  ros::Rate rate(100);
  while (!callback_executed && ros::ok())  // escape only when callback is triggered
  {
    publisher.publish(std_msgs::String());  // Publish msg to trigger callback
    ros::spinOnce();
    rate.sleep();
  }

  EXPECT_TRUE(callback_executed);  // Object's callback was executed by subscriber
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rostest");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}