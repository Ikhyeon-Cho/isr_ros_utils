/*
 * TransformHandler.h
 *
 *  Created on: Dec 19, 2022
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ROS_TRANSFORM_UTILS_TRANSFORM_HANDLER_H
#define ROS_TRANSFORM_UTILS_TRANSFORM_HANDLER_H

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ros
{
class TransformHandler
{
public:
  using Ptr = std::shared_ptr<TransformHandler>;

  /// @brief
  /// @param target_frame The frame to which the data should be transformed
  /// @param source_frame The frame where the data originated
  /// @param time Queried time for transform lookup
  /// @param timeout Time for waiting lookupTransform
  /// @param transform_stamped Transform (translation, rotation) from source to target
  /// @return True if succeed to get transform. False otherwise
  bool getTransform(const std::string& target_frame, const std::string& source_frame, const ros::Time& time,
                    const ros::Duration& timeout, geometry_msgs::TransformStamped& transform_stamped);

  /// @brief
  /// @param target_frame The frame to which the data should be transformed
  /// @param source_frame The frame where the data originated
  /// @param transform_stamped Transform (translation, rotation) from source to target
  /// @return True if succeed to get transform. False otherwise
  bool getTransform(const std::string& target_frame, const std::string& source_frame,
                    geometry_msgs::TransformStamped& transform_stamped);

  /// @brief
  /// @param target_frame The frame to which the data should be transformed
  /// @param source_frame The frame where the data originated
  /// @param time Queried time for transform lookup
  /// @param transform_stamped Transform (translation, rotation) from source to target
  /// @return True if succeed to get transform. False otherwise
  bool getTransform(const std::string& target_frame, const std::string& source_frame, const ros::Time& time,
                    geometry_msgs::TransformStamped& transform_stamped);

  /// @brief
  /// @param target_frame The frame to which the data should be transformed
  /// @param source_frame The frame where the data originated
  /// @param transform Eigen transform matrix from source to target
  /// @return True if succeed to get transform. False otherwise
  bool getTransformEigen(const std::string& target_frame, const std::string& source_frame, Eigen::Affine3d& transform);

  /// @brief
  /// @param target_frame The frame to which the data should be transformed
  /// @param source_frame The frame where the data originated
  /// @param time Queried time for transform lookup
  /// @param transform Eigen transform matrix from source to target
  /// @return True if succeed to get transform. False otherwise
  bool getTransformEigen(const std::string& target_frame, const std::string& source_frame, const ros::Time& time,
                         Eigen::Affine3d& transform);

  /// @brief
  /// @param target_frame The frame to which the data should be transformed
  /// @param source_frame The frame where the data originated
  /// @param time Queried time for transform lookup
  /// @param timeout Time for waiting lookupTransform
  /// @param transform Eigen transform matrix from source to target
  /// @return True if succeed to get transform. False otherwise
  bool getTransformEigen(const std::string& target_frame, const std::string& source_frame, const ros::Time& time,
                         const ros::Duration& timeout, Eigen::Affine3d& transform);

  void sendTransform(const geometry_msgs::Transform& transform, const std::string& target_frame,
                     const std::string& source_frame, const ros::Time& time);


  /// @brief
  /// @tparam T Datatype
  /// @param in The data to be transformed
  /// @param target_frame Target frame to be transformed
  /// @param source_frame Current frame of the data
  /// @param out A reference to the transformed data
  /// @return True if succeed to get transformed data. False otherwise
  template <typename T>
  inline bool doTransform(const T& in, const std::string& target_frame, const std::string& source_frame, T& out);

  /// @brief
  /// @tparam T Datatype
  /// @param in The data to be transformed
  /// @param target_frame Target frame to be transformed
  /// @param source_frame Current frame of the data
  /// @param time Queried time for transform lookup
  /// @param timeout Time for waiting lookupTransform
  /// @param out A reference to the transformed data
  /// @return True if succeed to get transformed data. False otherwise
  template <typename T>
  inline bool doTransform(const T& in, const std::string& target_frame, const std::string& source_frame,
                          const ros::Time& time, const ros::Duration& timeout, T& out);

  /// @brief
  /// @tparam T Datatype
  /// @param in The data to be transformed
  /// @param target_frame Target frame to be transformed
  /// @param time Queried time for transform lookup
  /// @param timeout Time for waiting lookupTransform
  /// @param out A reference to the transformed data
  /// @return True if succeed to get transformed data. False otherwise
  template <typename T>
  inline bool doTransform(const T& in, const std::string& target_frame, const ros::Time& time,
                          const ros::Duration& timeout, T& out)
  {
    return doTransform(in, target_frame, in.header.frame_id, time, timeout, out);
  }

  /// @brief
  /// @tparam T Datatype
  /// @param in The data to be transformed
  /// @param target_frame  Target frame to be transformed
  /// @param out A reference to the transformed data
  /// @return True if succeed to get transformed data. False otherwise
  template <typename T>
  inline bool doTransform(const T& in, const std::string& target_frame, T& out)
  {
    return doTransform(in, target_frame, in.header.frame_id, out);
  }

  /// @brief
  /// @param quaternion tf2 quaternion
  /// @param roll A reference of roll angle [rad]
  /// @param pitch A reference of pitch angle [rad]
  /// @param yaw A reference of yaw angle [rad]
  void getRPYFrom(const tf2::Quaternion& quaternion, double& roll, double& pitch, double& yaw);

  /// @brief
  /// @param quaternion geometry_msgs quaternion
  /// @param roll A reference of roll angle [rad]
  /// @param pitch A reference of pitch angle [rad]
  /// @param yaw A reference of yaw angle [rad]
  void getRPYFrom(const geometry_msgs::Quaternion& quaternion, double& roll, double& pitch, double& yaw);

  /// @brief
  /// @param roll roll angle [rad]
  /// @param pitch pitch angle [rad]
  /// @param yaw yaw angle [rad]
  /// @param quaternion A reference of tf2 quaternion
  void getQuaternionFrom(double roll, double pitch, double yaw, tf2::Quaternion& quaternion);

  /// @brief
  /// @param roll roll angle [rad]
  /// @param pitch pitch angle [rad]
  /// @param yaw yaw angle [rad]
  /// @param quaternion A reference of geometry_msgs quaternion
  void getQuaternionFrom(double roll, double pitch, double yaw, geometry_msgs::Quaternion& quaternion);

  /// @brief
  /// @param transform geometry_msgs::Transform Type
  /// @param transform_eigen Eigen Transform Matrix
  void toEigen(const geometry_msgs::Transform& transform, Eigen::Affine3d& transform_eigen);

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_{ std::make_shared<tf2_ros::Buffer>() };
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ std::make_shared<tf2_ros::TransformListener>(*tf_buffer_) };
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{ std::make_shared<tf2_ros::TransformBroadcaster>() };
};
// class TransformHandler

template <typename T>
bool TransformHandler::doTransform(const T& in, const std::string& target_frame, const std::string& source_frame,
                                   T& out)
{
  geometry_msgs::TransformStamped transform;
  if (!getTransform(target_frame, source_frame, transform))
    return false;

  tf2::doTransform(in, out, transform);
  return true;
}

template <typename T>
bool TransformHandler::doTransform(const T& in, const std::string& target_frame, const std::string& source_frame,
                                   const ros::Time& time, const ros::Duration& timeout, T& out)
{
  geometry_msgs::TransformStamped transform;
  if (!getTransform(target_frame, source_frame, time, timeout, transform))
    return false;

  tf2::doTransform(in, out, transform);
  return true;
}

}  // namespace ros

#endif  // ROS_TRANSFORM_UTILS_TRANSFORM_HANDLER_H