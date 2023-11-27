/*
 * msg_converter_ros.h
 *
 *  Created on: Sep 25, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ISR_ROS_MSG_CONVERTER
#define ISR_ROS_MSG_CONVERTER

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ros
{
class MsgConverter
{
public:
  /// @brief
  /// @param in geometry_msgs::PoseWithCovarianceStamped
  /// @return geometry_msgs::PoseStamped
  static geometry_msgs::PoseStamped toPoseStamped(const geometry_msgs::PoseWithCovarianceStamped& in);

  /// @brief
  /// @param in geometry_msgs::Quaternion
  /// @return tf2::Quaternion
  static tf2::Quaternion toQuaternionTF(const geometry_msgs::Quaternion& in);

  /// @brief
  /// @param in tf2::Quaternion
  /// @return geometry_msgs::Quaternion
  static geometry_msgs::Quaternion toQuaternionMsg(const tf2::Quaternion& in);

  /// @brief
  /// @param in tf2::Quaternion
  /// @param roll x-axis angle in degree
  /// @param pitch y-axis angle in degree
  /// @param yaw z-axis angle in degree
  static void fromQuaternionToRPY(const tf2::Quaternion& in, double& roll, double& pitch, double& yaw);

  /// @brief
  /// @param in geometry_msgs::Quaternion
  /// @param roll x-axis angle in degree
  /// @param pitch y-axis angle in degree
  /// @param yaw z-axis angle in degree
  static void fromQuaternionToRPY(const geometry_msgs::Quaternion& in, double& roll, double& pitch, double& yaw);

  /// @brief
  /// @param roll x-axis angle in degree
  /// @param pitch y-axis angle in degree
  /// @param yaw z-axis angle in degree
  /// @return geometry_msgs::Quaternion
  static geometry_msgs::Quaternion fromRPYToQuaternionMsg(double roll, double pitch, double yaw);

  /// @brief
  /// @param tf geometry_msgs::TransformStamped
  /// @return nav_msgs::Odometry
  static nav_msgs::Odometry toOdometry(const geometry_msgs::TransformStamped& tf);

  /// @brief
  /// @param tf geometry_msgs::TransformStamped
  /// @return geometry_msgs::PoseStamped
  static geometry_msgs::PoseStamped toPoseStamped(const geometry_msgs::TransformStamped& tf);

  /// @brief
  /// @param tf geometry_msgs::Transform
  /// @return geometry_msgs::Pose
  static geometry_msgs::Pose fromTransformToPose(const geometry_msgs::Transform& tf);

  /// @brief
  /// @param vector3 geometry_msgs::Vector3
  /// @return geometry_msgs::Point
  static geometry_msgs::Point fromVector3ToPoint(const geometry_msgs::Vector3& vector3);

  /// @brief
  /// @param point geometry_msgs::Point
  /// @return geometry_msgs::Vector3
  static geometry_msgs::Vector3 fromPointToVector3(const geometry_msgs::Point& point);
};
}  // namespace ros
#endif