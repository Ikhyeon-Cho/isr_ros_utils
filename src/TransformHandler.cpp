#include "isr_ros_utils/tools/TransformHandler.h"

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
                                       geometry_msgs::TransformStamped& transform_stamped)
{
  const ros::Time current_time(ros::Time(0));
  const ros::Duration timeout(ros::Duration(0.02));

  return getTransform(target_frame, source_frame, current_time, timeout, transform_stamped);
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

bool TransformHandler::getTransformEigen(const std::string& target_frame, const std::string& source_frame,
                                            Eigen::Affine3d& transform)
{
  geometry_msgs::TransformStamped transform_stamped;
  if (!getTransform(target_frame, source_frame, transform_stamped))
    return false;

  toEigen(transform_stamped.transform, transform);
  return true;
}

template <typename T>
bool TransformHandler::doTransform(const T& in, const std::string& target_frame, const std::string& source_frame,
                                      const ros::Time& time, const ros::Duration& timeout, T& out)
{
  geometry_msgs::TransformStamped transform;
  if (!getTransform(transform, target_frame, source_frame, time, timeout))
    return false;

  tf2::doTransform(in, out, transform);
  return true;
}

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
bool TransformHandler::doTransform(const T& in, const std::string& target_frame, const ros::Time& time,
                                      const ros::Duration& timeout, T& out)
{
  doTransform(in, target_frame, in.header.frame_id, time, timeout, out);
}

template <typename T>
bool TransformHandler::doTransform(const T& in, const std::string& target_frame, T& out)
{
  doTransform(in, target_frame, in.header.frame_id, out);
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

}  // namespace roscpp