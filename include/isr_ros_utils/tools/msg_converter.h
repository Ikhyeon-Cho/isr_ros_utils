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

namespace isr::roscpp
{
class MsgsConverter
{
public:
  static geometry_msgs::PoseStamped
  fromPoseWithCovStampedToPoseStamped(const geometry_msgs::PoseWithCovarianceStamped& _in)
  {
    geometry_msgs::PoseStamped out;
    out.header = _in.header;
    out.pose = fromPoseWithCovToPose(_in.pose);
    return out;
  }

  static geometry_msgs::Pose fromPoseWithCovToPose(const geometry_msgs::PoseWithCovariance& _in)
  {
    return _in.pose;
  }

  static tf2::Quaternion convertGeometryMsgsToTf2(const geometry_msgs::Quaternion& _in)
  {
    tf2::Quaternion out;
    tf2::fromMsg(_in, out);

    return out;
  }

  static geometry_msgs::Quaternion convertTf2ToGeometryMsgs(const tf2::Quaternion& _in)
  {
    return tf2::toMsg(_in);
  }

  static void getRPYFromQuat(const tf2::Quaternion& _in, double& roll, double& pitch, double& yaw)
  {
    tf2::Matrix3x3 matrix(_in);
    matrix.getEulerYPR(yaw, pitch, roll);
  }

  static void getRPYFromQuat(const geometry_msgs::Quaternion& _in, double& roll, double& pitch, double& yaw)
  {
    tf2::Quaternion quat = convertGeometryMsgsToTf2(_in);
    tf2::Matrix3x3 matrix(quat);
    matrix.getEulerYPR(yaw, pitch, roll);
  }

  static geometry_msgs::Quaternion getQuaternionFromRPY(double roll, double pitch, double yaw)
  {
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion out = convertTf2ToGeometryMsgs(quat);
    return out;
  }

  static nav_msgs::Odometry fromTransformStampedToOdometry(const geometry_msgs::TransformStamped& _tf)
  {
    nav_msgs::Odometry odom;
    odom.header = _tf.header;
    odom.child_frame_id = _tf.child_frame_id;
    odom.pose.pose = fromTransformToPose(_tf.transform);
    return odom;
  }

  static geometry_msgs::PoseStamped fromTransformStampedToPoseStamped(const geometry_msgs::TransformStamped& _tf)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = _tf.header;
    pose.pose = fromTransformToPose(_tf.transform);
    return pose;
  }

  static geometry_msgs::Pose fromTransformToPose(const geometry_msgs::Transform& _tf)
  {
    geometry_msgs::Pose pose;
    pose.position = fromVector3ToPoint(_tf.translation);
    pose.orientation = _tf.rotation;
    return pose;
  }

  static geometry_msgs::Point fromVector3ToPoint(const geometry_msgs::Vector3& _vector3)
  {
    geometry_msgs::Point point;
    point.x = _vector3.x;
    point.y = _vector3.y;
    point.z = _vector3.z;
    return point;
  }

  static geometry_msgs::Vector3 fromPointToVector3(const geometry_msgs::Point& _point)
  {
    geometry_msgs::Vector3 vector3;
    vector3.x = _point.x;
    vector3.y = _point.y;
    vector3.z = _point.z;
    return vector3;
  }
};
}  // namespace isr::navigation
#endif