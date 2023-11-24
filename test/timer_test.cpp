#include "isr_ros_utils/core/Timer.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <gtest/gtest.h>

namespace isr
{

class MockCallbackObject
{
public:
  void callback(const ros::TimerEvent& event)
  {
    callback_executed = true;
  }

  bool callback_executed{ false };
};

class TimerTest : public ::testing::Test
{
protected:
  MockCallbackObject object;
};

TEST_F(TimerTest, constructors)
{
  // timer 1
  roscpp::Timer timer(ros::Duration(0.1), &MockCallbackObject::callback, &object);
  EXPECT_DOUBLE_EQ(timer.getDuration(), 0.1);
  EXPECT_FALSE(timer.hasStarted());  // autostart = false

  // timer 2
  roscpp::Parameter<double> duration_param1{ "topic_key",
                                             0.1 };  // set to 0.1 since there is no pre-registered parameter
  roscpp::Timer timer2(duration_param1, &MockCallbackObject::callback, &object);
  EXPECT_DOUBLE_EQ(timer.getDuration(), 0.1);

  // timer 3
  ros::param::set("topic_key", 0.5);  // Register to ROS Parameter Server
  ros::Duration(0.1).sleep();
  roscpp::Parameter<double> duration_param2{ "topic_key", 0.1 };  // set to 0.5 since there is pre-registered parameter
  roscpp::Timer timer3(duration_param2, &MockCallbackObject::callback, &object);
  EXPECT_DOUBLE_EQ(timer3.getDuration(), 0.5);
}

TEST_F(TimerTest, timerStartCondition)
{
  // timer 1 with default option: manual start
  roscpp::Timer timer(ros::Duration(0.1), &MockCallbackObject::callback, &object);
  EXPECT_FALSE(timer.hasStarted());

  timer.start();
  EXPECT_TRUE(timer.hasStarted());

  // timer 2 with autostart option
  roscpp::Timer timer2(ros::Duration(0.1), &MockCallbackObject::callback, &object, true);
  EXPECT_TRUE(timer2.hasStarted());

  timer2.stop();
  EXPECT_FALSE(timer2.hasStarted());
}

TEST_F(TimerTest, timerCallback)
{
  // callback function in 'object' is triggered every 0.1 sec
  roscpp::Timer timer(0.1, &MockCallbackObject::callback, &object);
  EXPECT_FALSE(object.callback_executed);

  timer.start();

  ros::Rate rate(100);
  while (!object.callback_executed && ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  EXPECT_TRUE(object.callback_executed);
}

}  // namespace isr