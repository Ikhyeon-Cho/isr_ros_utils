/*
 * robot.h
 *
 *  Created on: Sep 19, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ISR_ROSCPP_TOOLS_ROBOT_ROS_H
#define ISR_ROSCPP_TOOLS_ROBOT_ROS_H

#include <Eigen/Core>

#include <isr_roscpp_core/publisher.h>
#include <isr_roscpp_tools/transform_handler.h>
#include "isr_ros_utils/tools/MsgConverter.h"
#include "isr_navigation/core/typedef.h"

#include <pcl/point_cloud.h>
#include <pcl/common/common.h>

namespace isr::navigation
{
class MobileRobot
{
public:
  using Ptr = std::shared_ptr<MobileRobot>;

private:
};

class DifferentialDriveRobot : public MobileRobot
{
public:
  using Ptr = std::shared_ptr<DifferentialDriveRobot>;

  using WheelVelocity = double;
  using WheelAcceleration = double;
  using WheelBase = double;
  using FootprintRadius = double;

protected:  // Pure >> reset in each robot class
  WheelVelocity maxWheelVelocity_;
  WheelAcceleration maxWheelAcceleration_;
  WheelBase wheelBaseLength_;

private:
  // ROS
  roscpp::TransformHandler transformHandler;
  nav_msgs::Odometry odometryPose_;  // Odometry pose
  geometry_msgs::PoseStamped pose_;  // Localized pose in map

  // FrameId
  roscpp::Parameter<std::string> mapFrame;
  roscpp::Parameter<std::string> odomFrame;
  roscpp::Parameter<std::string> baseFrame;

  // User-defined Limit
  roscpp::Parameter<double> maxLinearVelocity;
  roscpp::Parameter<double> maxAngularVelocity;
  roscpp::Parameter<double> footprintRadiusLength;

public:
  DifferentialDriveRobot()
    : maxWheelVelocity_(1.0)
    , maxWheelAcceleration_(1.5)
    , wheelBaseLength_(0.5)
    , transformHandler()
    , odometryPose_()
    , pose_()
  {
    mapFrame.readParameter("/FrameIds/map", "map");
    odomFrame.readParameter("/FrameIds/odom", "odom");
    baseFrame.readParameter("/FrameIds/base", "base_link");

    maxLinearVelocity.readParameter("/Robot/MaxLinearVelocity", 1.0);
    maxAngularVelocity.readParameter("/Robot/MaxAngularVelocity", 2.0);
    footprintRadiusLength.readParameter("/Robot/FootprintRadius", 0.5);
  }

  virtual ~DifferentialDriveRobot() = default;

  bool updatePose()
  {
    geometry_msgs::TransformStamped baseToOdom, baseToMap;
    if (!transformHandler.getTransform(baseToMap, mapFrame.value(), baseFrame.value()))
      return false;

    pose_ = ros::MsgConverter::toPoseStamped(baseToMap);
    return true;
  }

  bool updateOdometryPose()
  {
    geometry_msgs::TransformStamped baseToOdom;
    if (!transformHandler.getTransform(baseToOdom, odomFrame.value(), baseFrame.value()))
      return false;

    odometryPose_ = ros::MsgConverter::toOdometry(baseToOdom);
    return true;
  }

  controller::VlVr getWheelVelocityFromControlVelocity(const controller::CmdVel& cmd_vel) const
  {
    controller::VlVr wheel_vel;
    const auto& v = cmd_vel.x();
    const auto& w = cmd_vel.y();
    auto& v_l = wheel_vel.x();
    auto& v_r = wheel_vel.y();

    v_r = v + (wheelBaseLength_ * w) / 2;
    v_l = v - (wheelBaseLength_ * w) / 2;

    return wheel_vel;
  }

public:
  void setMaxLinearVelocity(double _maxV)
  {
    maxLinearVelocity.set(_maxV);
  }

  void setMaxAngularVelocity(double _maxW)
  {
    maxAngularVelocity.set(_maxW);
  }

  void setFrameId(const std::string& _frameId)
  {
    baseFrame.set(_frameId);
  }

  WheelVelocity getMaxWheelVelocity() const
  {
    return maxWheelVelocity_;
  }

  WheelAcceleration getMaxWheelAcceleration() const
  {
    return maxWheelAcceleration_;
  }

  WheelBase getWheelBaseLength() const
  {
    return wheelBaseLength_;
  }

  FootprintRadius getFootprintRadius() const
  {
    return footprintRadiusLength.value();
  }

  double getMaxLinearVelocity() const
  {
    return maxLinearVelocity.value();
  }

  double getMaxAngularVelocity() const
  {
    return maxAngularVelocity.value();
  }

  std::string getFrameId() const
  {
    return baseFrame.value();
  }

  const geometry_msgs::PoseStamped& getPose() const
  {
    return pose_;
  }

  const nav_msgs::Odometry& getOdometryPose() const
  {
    return odometryPose_;
  }
};

class ISR_M2 : public DifferentialDriveRobot
{
private:
  roscpp::Publisher<nav_msgs::Odometry> odomPublisher;
  roscpp::Publisher<geometry_msgs::PoseStamped> posePublisher;

public:
  using Ptr = std::shared_ptr<ISR_M2>;

  ISR_M2(const ros::NodeHandle& _nh)
    : DifferentialDriveRobot(), odomPublisher(_nh, "/isr_m2/odom"), posePublisher(_nh, "/isr_m2/pose")

  {
    maxWheelVelocity_ = 0.8;
    maxWheelAcceleration_ = 2.0;
    wheelBaseLength_ = 0.6;

    odomPublisher.registerPublisher(10);
    posePublisher.registerPublisher(10);
  }
};

class ISR_M3 : public DifferentialDriveRobot
{
private:
  roscpp::Publisher<nav_msgs::Odometry> odomPublisher;
  roscpp::Publisher<geometry_msgs::PoseStamped> posePublisher;

public:
  using Ptr = std::shared_ptr<ISR_M3>;

  ISR_M3(const ros::NodeHandle& _nh)
    : DifferentialDriveRobot(), odomPublisher(_nh, "/isr_m3/odom"), posePublisher(_nh, "/isr_m3/pose")
  {
    maxWheelVelocity_ = 0.8;
    maxWheelAcceleration_ = 2.0;
    wheelBaseLength_ = 0.6;

    odomPublisher.registerPublisher(10);
    posePublisher.registerPublisher(10);
  }
};

}  // namespace isr::navigation

#endif