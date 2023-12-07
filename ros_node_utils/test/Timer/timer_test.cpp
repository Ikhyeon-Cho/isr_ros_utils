#include "ros_node_utils/Timer.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <gtest/gtest.h>

class TimerTest : public ::testing::Test
{
protected:
  void callback(const ros::TimerEvent& event)
  {
    callback_executed = true;
  }

  bool callback_executed{ false };

  // 1. Timer initialized with ros::Duration. Every duration, callback is triggered
  roscpp::Timer timer1_{ ros::Duration(0.1), &TimerTest::callback, this };

  // 2. Timer has auto-start option. In default, autostart option is false.
  roscpp::Timer timer2_{ ros::Duration(0.3), &TimerTest::callback, this, true };

  // 3. Timer can use roscpp::Parameter to initialize with parameterized time duration
  roscpp::Parameter<double> duration_param{ "duration_key", 0.1 };  // in test launch, parameter is set to 0.5
  roscpp::Timer timer3_{ duration_param, &TimerTest::callback, this };
};

TEST_F(TimerTest, timerStartCondition)
{
  // timer 1
  EXPECT_DOUBLE_EQ(timer1_.getDuration(), 0.1);
  EXPECT_FALSE(timer1_.isRunning());  // You need to manually start the timer in default
  timer1_.start();
  EXPECT_TRUE(timer1_.isRunning());

  // timer 2
  EXPECT_DOUBLE_EQ(timer2_.getDuration(), 0.3);
  EXPECT_TRUE(timer2_.isRunning());  // With autostart option, timer is running right after construction
  timer2_.stop();
  EXPECT_FALSE(timer2_.isRunning());  // Timer can be manually stopped

  // timer 3
  EXPECT_DOUBLE_EQ(timer3_.getDuration(), 0.5);
}

TEST_F(TimerTest, callback)
{
  // callback function in 'object' is triggered every 0.1 sec
  EXPECT_FALSE(callback_executed);

  timer1_.start();

  ros::Rate rate(100);
  while (!callback_executed && ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  EXPECT_TRUE(callback_executed);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rostest");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}