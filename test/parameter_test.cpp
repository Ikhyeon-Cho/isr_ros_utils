#include "isr_ros_utils/core/parameter.h"

#include <ros/ros.h>
#include <gtest/gtest.h>

namespace isr
{
class ParameterTest : public ::testing::Test
{
protected:
  roscpp::Parameter<int> param_int_;
  roscpp::Parameter<float> param_float_;
  roscpp::Parameter<double> param_double_;
  roscpp::Parameter<std::string> param_string_;
};

//! Note default values
TEST_F(ParameterTest, constructor)
{
  EXPECT_EQ(param_int_.value(), 0);
  EXPECT_FLOAT_EQ(param_float_.value(), 0.f);
  EXPECT_DOUBLE_EQ(param_double_.value(), 0.0);
  EXPECT_STREQ(param_string_.value().c_str(), "");
}

TEST_F(ParameterTest, defaultValueConstructor)
{
  roscpp::Parameter<int> param_int(42);
  roscpp::Parameter<float> param_float(42);
  roscpp::Parameter<double> param_double(42);
  roscpp::Parameter<std::string> param_string("test");

  EXPECT_EQ(param_int.value(), 42);
  EXPECT_FLOAT_EQ(param_float.value(), 42.f);
  EXPECT_DOUBLE_EQ(param_double.value(), 42.0);
  EXPECT_STREQ(param_string.value().c_str(), "test");
}

TEST_F(ParameterTest, readParamConstructor)
{
  // Register value to Parameter Server
  ros::param::set("param_int", 100);
  ros::param::set("param_float", 100.f);
  ros::param::set("param_double", 100.0);
  ros::param::set("param_string", "100");

  // Read from Parameter Server with Constructor
  roscpp::Parameter<int> param_int("param_int", 42);
  roscpp::Parameter<float> param_float("param_float", 42);
  roscpp::Parameter<double> param_double("param_double", 42);
  roscpp::Parameter<std::string> param_string("param_string" ,"test");

  EXPECT_EQ(param_int.value(), 100);
  EXPECT_FLOAT_EQ(param_float.value(), 100);
  EXPECT_DOUBLE_EQ(param_double.value(), 100);
  EXPECT_STREQ(param_string.value().c_str(), "100");
}

TEST_F(ParameterTest, setValue)
{
  param_int_.set(20);
  param_float_.set(20);
  param_double_.set(20);
  param_string_.set("set_value");

  EXPECT_EQ(param_int_.value(), 20);
  EXPECT_FLOAT_EQ(param_float_.value(), 20.f);
  EXPECT_DOUBLE_EQ(param_double_.value(), 20.0);
  EXPECT_STREQ(param_string_.value().c_str(), "set_value");
}

TEST_F(ParameterTest, readParameter)
{
  // Register to ROS Parameter Server
  ros::param::set("/my_parameter_number", 30);

  // Read from Parameter Server
  bool success_int_1 = param_int_.readParameter("/my_parameter_number");
  bool success_float_1 = param_float_.readParameter("/my_parameter_number");
  bool success_double_1 = param_double_.readParameter("/my_parameter_number");
  bool success_string_1 = param_string_.readParameter("/my_parameter_number");

  // Check success of readParameter function
  EXPECT_TRUE(success_int_1);
  EXPECT_TRUE(success_float_1);
  EXPECT_TRUE(success_double_1);
  EXPECT_FALSE(success_string_1);  // Only False in string type

  // Expected Values after readParameter
  EXPECT_EQ(param_int_.value(), 30);
  EXPECT_FLOAT_EQ(param_float_.value(), 30.f);
  EXPECT_DOUBLE_EQ(param_double_.value(), 30.0);
  EXPECT_STREQ(param_string_.value().c_str(), "");  // Does not change value

  ros::param::set("/my_parameter_string", "param");

  bool success_int_2 = param_int_.readParameter("/my_parameter_string");
  bool success_float_2 = param_float_.readParameter("/my_parameter_string");
  bool success_double_2 = param_double_.readParameter("/my_parameter_string");
  bool success_string_2 = param_string_.readParameter("/my_parameter_string");

  EXPECT_FALSE(success_int_2);
  EXPECT_FALSE(success_float_2);
  EXPECT_FALSE(success_double_2);
  EXPECT_TRUE(success_string_2);  // Only True in string type

  EXPECT_EQ(param_int_.value(), 30);
  EXPECT_FLOAT_EQ(param_float_.value(), 30.f);
  EXPECT_DOUBLE_EQ(param_double_.value(), 30.0);
  EXPECT_STREQ(param_string_.value().c_str(), "param");  // Only changed in here
}

TEST_F(ParameterTest, readParameterWithDefault)
{
  // Read from Parameter Server
  bool success_int = param_int_.readParameter("/nonexistent_parameter");
  bool success_float = param_float_.readParameter("/nonexistent_parameter");
  bool success_double = param_double_.readParameter("/nonexistent_parameter");
  bool success_string = param_string_.readParameter("/nonexistent_parameter");

  // Check success of readParameter function
  EXPECT_FALSE(success_int);
  EXPECT_FALSE(success_float);
  EXPECT_FALSE(success_double);
  EXPECT_FALSE(success_string);

  // Expected Values after readParameter: remain default
  EXPECT_EQ(param_int_.value(), 0);
  EXPECT_FLOAT_EQ(param_float_.value(), 0.f);
  EXPECT_DOUBLE_EQ(param_double_.value(), 0.0);
  EXPECT_STREQ(param_string_.value().c_str(), "");  // Only changed in here
}

TEST_F(ParameterTest, get)
{
  roscpp::Parameter<int> param_int(1);
  roscpp::Parameter<float> param_float(2);
  roscpp::Parameter<double> param_double(3);
  roscpp::Parameter<std::string> param_string("four");

  // Expected Values after get: remain default
  EXPECT_EQ(param_int.get(), 1);
  EXPECT_FLOAT_EQ(param_float.get(), 2.f);
  EXPECT_DOUBLE_EQ(param_double.get(), 3.0);
  EXPECT_STREQ(param_string.get().c_str(), "four");
}

}  // namespace isr