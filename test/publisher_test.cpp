#include "isr_ros_utils/core/publisher.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <gtest/gtest.h>

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

TEST_F(PublisherTest, Constructors)
{
  // // Check topic settings
  EXPECT_STREQ(local_topic_publisher_.getTopic().c_str(), "/test_node/topic");  // Default: local namespace ("~")
  EXPECT_STREQ(global_topic_publisher_.getTopic().c_str(), "/topic");           // global namespace ("/")
  EXPECT_STREQ(new_namespace_topic_publisher_.getTopic().c_str(), "/new_namespace/topic");  // new namespace

  // Check queue size
  EXPECT_EQ(local_topic_publisher_.getQueueSize(), 10);
  EXPECT_EQ(global_topic_publisher_.getQueueSize(), 5);
  EXPECT_EQ(new_namespace_topic_publisher_.getQueueSize(), 10);
}

TEST_F(PublisherTest, ReadParameter)
{
  // Read from Parameter Server
  bool success_local = local_topic_publisher_.readParameter("topic");
  bool success_global = global_topic_publisher_.readParameter("topic");
  bool success_new = new_namespace_topic_publisher_.readParameter("topic");

  // Check parameters were read successfully
  EXPECT_TRUE(success_local);
  EXPECT_TRUE(success_global);
  EXPECT_TRUE(success_new);

  // Expected Values after readParameter
  EXPECT_STREQ(local_topic_publisher_.getTopic().c_str(), "/test_node/topic_from_server");
  EXPECT_STREQ(global_topic_publisher_.getTopic().c_str(), "/test_node/topic_from_server");

  EXPECT_STREQ(new_namespace_topic_publisher_.getTopic().c_str(), "/new_namespace/topic_from_server");

  //! Note: readParameter use global namespace as a default. Local should be explicitly defined
  bool success_test = local_topic_publisher_.readParameter("~/topic");
  EXPECT_FALSE(success_test);
}

}  // namespace isr