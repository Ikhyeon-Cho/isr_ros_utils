/*
 * TransformHandler.h
 *
 *  Created on: Dec 19, 2022
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "ros_transform_utils/TransformHandler.h"

namespace ros
{
bool TransformHandler::getTransform(const std::string& target_frame, const std::string& source_frame,
                                    const ros::Time& time, const ros::Duration& timeout,
                                    geometry_msgs::TransformStamped& transform_stamped)
{
  try
  {
    transform_stamped = tf_buffer_->lookupTransform(target_frame, source_frame, time, timeout);
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_ERROR_STREAM_THROTTLE(1, "Failed to look up transform from " << source_frame << " to " << target_frame << ": "
                                                                     << ex.what());  // 0.2us
    return false;
  }
}

bool TransformHandler::getTransform(const std::string& target_frame, const std::string& source_frame,
                                    const ros::Time& time, geometry_msgs::TransformStamped& transform_stamped)
{
  const ros::Duration timeout(ros::Duration(0.02));
  return getTransform(target_frame, source_frame, time, timeout, transform_stamped);
}

bool TransformHandler::getTransform(const std::string& target_frame, const std::string& source_frame,
                                    geometry_msgs::TransformStamped& transform_stamped)
{
  const ros::Time current_time(ros::Time(0));
  const ros::Duration timeout(ros::Duration(0.02));

  return getTransform(target_frame, source_frame, current_time, timeout, transform_stamped);
}

bool TransformHandler::getTransformEigen(const std::string& target_frame, const std::string& source_frame,
                                         Eigen::Affine3d& transform)
{
  geometry_msgs::TransformStamped transform_stamped;
  if (!getTransform(target_frame, source_frame, transform_stamped))
    return false;

  toEigen(transform_stamped.transform, transform);
  return true;
}

bool TransformHandler::getTransformEigen(const std::string& target_frame, const std::string& source_frame,
                                         const ros::Time& time, Eigen::Affine3d& transform)
{
  geometry_msgs::TransformStamped transform_stamped;
  if (!getTransform(target_frame, source_frame, time, transform_stamped))
    return false;

  toEigen(transform_stamped.transform, transform);
  return true;
}

bool TransformHandler::getTransformEigen(const std::string& target_frame, const std::string& source_frame,
                                         const ros::Time& time, const ros::Duration& timeout,
                                         Eigen::Affine3d& transform)
{
  geometry_msgs::TransformStamped transform_stamped;
  if (!getTransform(target_frame, source_frame, time, timeout, transform_stamped))
    return false;

  toEigen(transform_stamped.transform, transform);
  return true;
}

void TransformHandler::sendTransform(const geometry_msgs::Transform& transform, const std::string& target_frame,
                                     const std::string& source_frame, const ros::Time& time)
{
  geometry_msgs::TransformStamped tf;
  tf.header.stamp = time;
  tf.header.frame_id = target_frame;
  tf.child_frame_id = source_frame;
  tf.transform = transform;

  tf_broadcaster_->sendTransform(tf);
}

void TransformHandler::getRPYFrom(const tf2::Quaternion& quaternion, double& roll, double& pitch, double& yaw)
{
  tf2::Matrix3x3 matrix(quaternion);
  matrix.getEulerYPR(yaw, pitch, roll);
}

void TransformHandler::getRPYFrom(const geometry_msgs::Quaternion& quaternion, double& roll, double& pitch, double& yaw)
{
  tf2::Quaternion quaternion_tf;
  tf2::fromMsg(quaternion, quaternion_tf);
  getRPYFrom(quaternion_tf, roll, pitch, yaw);
}

void TransformHandler::getQuaternionFrom(double roll, double pitch, double yaw, tf2::Quaternion& quaternion)
{
  quaternion.setRPY(roll, pitch, yaw);
}

void TransformHandler::getQuaternionFrom(double roll, double pitch, double yaw, geometry_msgs::Quaternion& quaternion)
{
  tf2::Quaternion quaternion_tf;
  quaternion_tf.setRPY(roll, pitch, yaw);
  quaternion = tf2::toMsg(quaternion_tf);
}

void TransformHandler::toEigen(const geometry_msgs::Transform& transform, Eigen::Affine3d& transform_eigen)
{
  transform_eigen = Eigen::Affine3d::Identity();

  // set translation
  Eigen::Vector3d translation;
  translation << transform.translation.x, transform.translation.y, transform.translation.z;
  transform_eigen.translation() = translation;

  // set rotation
  Eigen::Quaterniond rotation;
  rotation.x() = transform.rotation.x;
  rotation.y() = transform.rotation.y;
  rotation.z() = transform.rotation.z;
  rotation.w() = transform.rotation.w;
  transform_eigen.rotate(rotation);
}

}  // namespace ros