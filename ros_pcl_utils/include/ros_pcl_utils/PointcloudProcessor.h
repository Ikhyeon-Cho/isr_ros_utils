/*
 * PointcloudProcessor.h
 *
 *  Created on: Apr 22, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ROS_PCL_UTILS_POINTCLOUD_PROCESSOR_H
#define ROS_PCL_UTILS_POINTCLOUD_PROCESSOR_H

// PCL
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

// ROS
#include <pcl_conversions/pcl_conversions.h>

#include <ros_transform_utils/TransformHandler.h>

namespace ros
{
template <typename T>
class PointcloudProcessor
{
public:
  typedef pcl::PointCloud<T> PointCloud;
  typedef typename pcl::PointCloud<T>::Ptr PointCloudPtr;

public:
  PointcloudProcessor();

  /// @brief The users should check whether the transform is succeeded or not (nullptr)
  /// @param input The pointcloud to be transformed
  /// @param target_frame The frame to which the data should be transformed
  /// @param success True if succeed to get transform. False otherwise
  /// @return A shared_ptr of the transformed data. If fails to get transform, returns nullptr
  PointCloudPtr transformPointcloud(const PointCloudPtr& input, const std::string& target_frame, bool& success);

  PointCloudPtr filterPointcloudByAxis(const PointCloudPtr& input, const std::string& axis, double range_min,
                                       double range_max, bool negative = false);

  PointCloudPtr filterPointcloudByRange2D(const PointCloudPtr& input, double range_min, double range_max);

  PointCloudPtr filterPointcloudByRange(const PointCloudPtr& input, double range_min, double range_max);

  PointCloudPtr filterPointcloudByAngle(const PointCloudPtr& input, double angle_start, double angle_end,
                                        bool negative = false);

  PointCloudPtr filterPointcloudByVoxel(const PointCloudPtr& input, double voxel_x, double voxel_y, double voxel_z);

  PointCloudPtr filterPointcloudByVoxel(const PointCloudPtr& input, double voxel_size);

  // TODO: Move to velodyne-specific template class
  void filterRingOfIndex(double _ringIndex, bool _negative = false);

private:
  float pointDistance(const T& p)
  {
    return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
  };

private:
  TransformHandler transform_handler_;
  PointCloudPtr pclCloud_;
};

template <typename T>
PointcloudProcessor<T>::PointcloudProcessor()
{
  pclCloud_ = boost::make_shared<PointCloud>();
}

template <typename T>
typename PointcloudProcessor<T>::PointCloudPtr
PointcloudProcessor<T>::transformPointcloud(const PointCloudPtr& input, const std::string& target_frame, bool& success)
{
  std::string source_frame(input->header.frame_id);
  if (source_frame.empty())
  {
    ROS_ERROR_STREAM(" [PointcloudProcessor] Warning: Transform failure -  pointcloud has no frame id");
    success = false;
    return nullptr;
  }

  if (source_frame == target_frame)
  {
    success = true;
    return input;
  }

  Eigen::Affine3d transform_matrix;
  ros::Time timestamp = pcl_conversions::fromPCL(input->header.stamp);
  if (!transform_handler_.getTransformEigen(target_frame, source_frame, timestamp, transform_matrix))
  {
    success = false;
    return nullptr;
  }

  PointCloudPtr output = boost::make_shared<PointCloud>();
  pcl::transformPointCloud(*input, *output, transform_matrix);
  output->header = input->header;
  output->header.frame_id = target_frame;
  success = true;
  return output;
}

template <typename T>
typename PointcloudProcessor<T>::PointCloudPtr PointcloudProcessor<T>::filterPointcloudByAxis(
    const PointCloudPtr& input, const std::string& axis, double range_min, double range_max, bool negative)
{
  if (input->empty())
    return input;

  PointCloudPtr output = boost::make_shared<PointCloud>();

  pcl::PassThrough<T> ps;
  ps.setInputCloud(input);
  ps.setFilterFieldName(axis);
  ps.setFilterLimits(range_min, range_max);
  ps.setFilterLimitsNegative(negative);
  ps.filter(*output);
  output->header = input->header;
  return output;
}

template <typename T>
typename PointcloudProcessor<T>::PointCloudPtr
PointcloudProcessor<T>::filterPointcloudByRange2D(const PointCloudPtr& input, double range_min, double range_max)
{
  if (input->empty())
    return input;

  PointCloudPtr output = boost::make_shared<PointCloud>();
  for (auto point : input->points)
  {
    double range_2D = sqrt(point.x * point.x + point.y * point.y);

    if (range_2D > range_min && range_2D < range_max)
    {
      T filtered_point = point;
      filtered_point.intensity = range_2D;
      output->points.push_back(filtered_point);
    }
  }
  output->header = input->header;
  return output;
}

template <typename T>
typename PointcloudProcessor<T>::PointCloudPtr
PointcloudProcessor<T>::filterPointcloudByRange(const PointCloudPtr& input, double range_min, double range_max)
{
  if (input->empty())
    return input;

  PointCloudPtr output = boost::make_shared<PointCloud>();
  for (auto point : input->points)
  {
    double range_2D = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

    if (range_2D > range_min && range_2D < range_max)
    {
      T filtered_point = point;
      filtered_point.intensity = range_2D;
      output->points.push_back(filtered_point);
    }
  }
  output->header = input->header;
  return output;
}

template <typename T>
typename PointcloudProcessor<T>::PointCloudPtr PointcloudProcessor<T>::filterPointcloudByAngle(
    const PointCloudPtr& input, double angle_start, double angle_end, bool negative)
{
  if (input->empty())
    return input;

  PointCloudPtr output = boost::make_shared<PointCloud>();
  for (auto point : input->points)
  {
    double angle_horizon = std::atan2(point.y, point.x);
    bool filter_condition = angle_horizon > DEG2RAD(angle_start) && angle_horizon < DEG2RAD(angle_end);
    if (negative)
      filter_condition = !filter_condition;

    if (filter_condition)
    {
      T filtered_point = point;
      filtered_point.intensity = pointDistance(point);
      output->points.push_back(filtered_point);
    }
  }
  output->header = input->header;
  return output;
}

template <typename T>
typename PointcloudProcessor<T>::PointCloudPtr PointcloudProcessor<T>::filterPointcloudByVoxel(
    const PointCloudPtr& input, double voxel_x, double voxel_y, double voxel_z)
{
  if (input->empty())
    return input;

  PointCloudPtr output = boost::make_shared<PointCloud>();
  pcl::VoxelGrid<T> vox;
  vox.setInputCloud(input);
  vox.setLeafSize(voxel_x, voxel_y, voxel_z);
  vox.filter(*output);
  output->header = input->header;
  return output;
}

template <typename T>
typename PointcloudProcessor<T>::PointCloudPtr
PointcloudProcessor<T>::filterPointcloudByVoxel(const PointCloudPtr& input, double voxel_size)
{
  return filterPointcloudByVoxel(input, voxel_size, voxel_size, voxel_size);
}

template <typename T>
void PointcloudProcessor<T>::filterRingOfIndex(double _ringIndex, bool _negative)
{
  PointCloudPtr filteredCloud = boost::make_shared<PointCloud>();
  filteredCloud->header = pclCloud_->header;
  filteredCloud->points.resize(pclCloud_->size());
  filteredCloud->clear();

  for (auto& point : pclCloud_->points)
  {
    if ((point.ring == _ringIndex) ^ _negative)
    {
      auto point_new = point;
      point_new.intensity = pointDistance(point);
      filteredCloud->points.push_back(point_new);
    }
  }

  pclCloud_.swap(filteredCloud);
}

}  // namespace ros

#endif  // ROS_PCL_UTILS_POINTCLOUD_PROCESSOR_HH