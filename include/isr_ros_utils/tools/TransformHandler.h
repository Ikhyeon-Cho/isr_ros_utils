/*
 * TransformHandler.h
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

namespace ros
{
class TransformHandler
{
public:
  using Ptr = std::shared_ptr<TransformHandler>;

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
  /// @param timeout Time for waiting lookupTransform
  /// @param transform_stamped Transform (translation, rotation) from source to target
  /// @return  True if succeed to get transform. False otherwise
  bool getTransform(const std::string& target_frame, const std::string& source_frame, const ros::Time& time,
                    const ros::Duration& timeout, geometry_msgs::TransformStamped& transform_stamped);

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
  /// @param timeout Time for waiting lookupTransform
  /// @param transform Eigen transform matrix from source to target
  /// @return True if succeed to get transform. False otherwise
  bool getTransformEigen(const std::string& target_frame, const std::string& source_frame, const ros::Time& time,
                         const ros::Duration& timeout, Eigen::Affine3d& transform);

  /// @brief
  /// @tparam T Datatype
  /// @param in The data to be transformed
  /// @param target_frame Target frame to be transformed
  /// @param source_frame Current frame of the data
  /// @param out A reference to the transformed data
  /// @return True if succeed to get transformed data. False otherwise
  template <typename T>
  bool doTransform(const T& in, const std::string& target_frame, const std::string& source_frame, T& out);

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
  bool doTransform(const T& in, const std::string& target_frame, const std::string& source_frame, const ros::Time& time,
                   const ros::Duration& timeout, T& out);

  /// @brief
  /// @tparam T Datatype
  /// @param in The data to be transformed
  /// @param target_frame Target frame to be transformed
  /// @param time Queried time for transform lookup
  /// @param timeout Time for waiting lookupTransform
  /// @param out A reference to the transformed data
  /// @return True if succeed to get transformed data. False otherwise
  template <typename T>
  bool doTransform(const T& in, const std::string& target_frame, const ros::Time& time, const ros::Duration& timeout,
                   T& out);

  /// @brief
  /// @tparam T Datatype
  /// @param in The data to be transformed
  /// @param target_frame  Target frame to be transformed
  /// @param out A reference to the transformed data
  /// @return True if succeed to get transformed data. False otherwise
  template <typename T>
  bool doTransform(const T& in, const std::string& target_frame, T& out);

private:
  void toEigen(const geometry_msgs::Transform& transform, Eigen::Affine3d& transform_eigen);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_{ std::make_shared<tf2_ros::Buffer>() };
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ std::make_shared<tf2_ros::TransformListener>(*tf_buffer_) };
};
// class TransformHandler

}  // namespace roscpp

#endif  // ISR_ROSCPP_TOOLS_TRANSFORM_HANDLER_H