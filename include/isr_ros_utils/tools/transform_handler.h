/*
 * transform_handler.h
 *
 *  Created on: Dec 19, 2022
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ISR_ROSCPP_TOOLS_TRANSFORM_HANDLER_H
#define ISR_ROSCPP_TOOLS_TRANSFORM_HANDLER_H

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace isr::roscpp
{
class TransformHandler
{
public:
  using Ptr = std::shared_ptr<TransformHandler>;

  TransformHandler() = default;

  /*
   * \param transform_stamped Transform from source frame to target frame
   * \param target_frame New reference frame
   * \param source_frame Frame to be transformed
   * \param time Query time for getting transform
   * \param timeout Time for waiting lookupTransform
   * **/
  bool getTransform(geometry_msgs::TransformStamped& transform_stamped, const std::string& target_frame,
                    const std::string& source_frame, const ros::Time& time = ros::Time(0),
                    const ros::Duration timeout = ros::Duration(0.01))
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

  /*
   * \param in Data to be transformed
   * \param target_frame New reference frame
   * \param source_frame Frame to be transformed
   * \param time Query time for getting transform
   * \param timeout Time for waiting lookupTransform
   * **/
  template <class T>
  std::tuple<T, bool> doTransform(const T& in, const std::string& target_frame, const std::string& source_frame,
                                  const ros::Time& time = ros::Time(0), ros::Duration timeout = ros::Duration(0.1));

  /*
   * \param in Data to be transformed
   * \param target_frame New reference frame
   * \param time Query time for getting transform
   * \param timeout Time for waiting lookupTransform
   * **/
  template <class T>
  std::tuple<T, bool> doTransform(const T& in, const std::string& target_frame, const ros::Time& time = ros::Time(0),
                                  ros::Duration timeout = ros::Duration(0.1));

  /*
   * \param transform Msg type of geometry_msgs::Transform
   * **/
  Eigen::Affine3d toEigen(const geometry_msgs::Transform& transform);

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_{ std::make_shared<tf2_ros::Buffer>() };
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ std::make_shared<tf2_ros::TransformListener>(*tf_buffer_) };
};
// class TransformHandler

// bool TransformHandler::getTransform(geometry_msgs::TransformStamped& transform_stamped, const std::string&
// target_frame,
//                                     const std::string& source_frame, const ros::Time& time, const ros::Duration
//                                     timeout)
// {
//   try
//   {
//     transform_stamped = tf_buffer_->lookupTransform(target_frame, source_frame, time, timeout);
//     return true;
//   }
//   catch (const tf2::TransformException& ex)
//   {
//     ROS_ERROR_STREAM_THROTTLE(1, "Failed to look up transform from " << source_frame << " to " << target_frame << ":
//     "
//                                                                      << ex.what());  // 0.2us
//     return false;
//   }
// }

template <typename T>
inline std::tuple<T, bool> TransformHandler::doTransform(const T& in, const std::string& target_frame,
                                                         const std::string& source_frame, const ros::Time& time,
                                                         ros::Duration timeout)
{
  T transformed_data;

  geometry_msgs::TransformStamped transform;
  if (!getTransform(transform, target_frame, source_frame, time, timeout))
    return { transformed_data, false };

  tf2::doTransform(in, transformed_data, transform);
  return { transformed_data, true };
}

template <typename T>
inline std::tuple<T, bool> TransformHandler::doTransform(const T& in, const std::string& target_frame,
                                                         const ros::Time& time, ros::Duration timeout)
{
  const auto& source_frame_id = in.header.frame_id;
  if (source_frame_id.empty())
  {
    ROS_WARN("Transform Data failed because of empty frame id.");
    return { T(), false };
  }

  return doTransform(in, target_frame, source_frame_id, time, timeout);
}

inline Eigen::Affine3d TransformHandler::toEigen(const geometry_msgs::Transform& transform)
{
  Eigen::Affine3d eigen(Eigen::Affine3d::Identity());

  // set translation
  Eigen::Vector3d translation;
  translation << transform.translation.x, transform.translation.y, transform.translation.z;
  eigen.translation() = translation;

  // set rotation
  Eigen::Quaterniond rotation;
  rotation.x() = transform.rotation.x;
  rotation.y() = transform.rotation.y;
  rotation.z() = transform.rotation.z;
  rotation.w() = transform.rotation.w;
  eigen.rotate(rotation);

  return eigen;
}

}  // namespace isr::roscpp

#endif  // ISR_ROSCPP_TOOLS_TRANSFORM_HANDLER_H