/*
 * PointcloudProcessor.h
 *
 *  Created on: Apr 22, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ISR_CLOUD_PROCESSOR_ROS_H
#define ISR_CLOUD_PROCESSOR_ROS_H

// PCL
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

// Custom point type
#include "velodyne_point.h"

// ROS
#include "isr_ros_utils/tools/TransformHandler.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// Use the Velodyne point format as a common representation

namespace ros
{
template <class T>
class PointcloudProcessor
{
public:
  typedef pcl::PointCloud<T> PointCloud;
  typedef typename pcl::PointCloud<T>::Ptr PointCloudPtr;

public:
  PointcloudProcessor();

  void setPointCloud(const sensor_msgs::PointCloud2& _cloud);

  void getPointCloud(sensor_msgs::PointCloud2& _cloud);

  sensor_msgs::PointCloud2Ptr getPointCloud();

  void clearPointCloud();

  bool transformPointCloudTo(const std::string& _target_frame);

  void filterCloudByRange(double _minRange, double _maxRange);

  void filterCloudByAngle(double _filterAngle);

  void filterCloudByAxis(const std::string& _axis, double _minRange, double _maxRange, bool _negative = false);

  void voxelDownsampling(double _voxelLeafSize);

  void filterRingOfIndex(double _ringIndex, bool _negative = false);

private:
  bool cloudIsEmpty();

  float pointDistance(const T& p)
  {
    return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
  };

private:
  TransformHandler transform_handler_;
  PointCloudPtr pclCloud_;
};

template <class T>
PointcloudProcessor<T>::PointcloudProcessor()
{
  pclCloud_ = boost::make_shared<PointCloud>();
}

template <class T>
void PointcloudProcessor<T>::setPointCloud(const sensor_msgs::PointCloud2& _msg)
{
  pclCloud_->clear();
  pcl::fromROSMsg(_msg, *pclCloud_);
}

template <class T>
void PointcloudProcessor<T>::getPointCloud(sensor_msgs::PointCloud2& _cloud)
{
  pcl::toROSMsg(*pclCloud_, _cloud);
}

template <class T>
sensor_msgs::PointCloud2Ptr PointcloudProcessor<T>::getPointCloud()
{
  sensor_msgs::PointCloud2Ptr cloudPtr = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(*pclCloud_, *cloudPtr);
  return cloudPtr;
}

template <class T>
void PointcloudProcessor<T>::clearPointCloud()
{
  pclCloud_->clear();
}

template <class T>
bool PointcloudProcessor<T>::transformPointCloudTo(const std::string& target_frame)
{
  std::string source_frame = pclCloud_->header.frame_id;
  if (source_frame.empty())
  {
    ROS_ERROR_STREAM("cloud cannot be transformed because it has no frame id");
    return false;
  }

  if (source_frame == target_frame)
    return true;

  Eigen::Affine3d transform_matrix;
  ros::Time timestamp = pcl_conversions::fromPCL(pclCloud_->header.stamp);
  if (!transform_handler_.getTransformEigen(target_frame, source_frame, timestamp, transform_matrix))
    return false;

  PointCloudPtr transformedCloud = boost::make_shared<PointCloud>();
  transformedCloud->header = pclCloud_->header;

  pcl::transformPointCloud(*pclCloud_, *transformedCloud, transform_matrix);
  pclCloud_.swap(transformedCloud);
  pclCloud_->header.frame_id = target_frame;

  return true;
}

template <class T>
void PointcloudProcessor<T>::filterCloudByRange(double _minRange, double _maxRange)
{
  if (cloudIsEmpty())
    return;

  PointCloudPtr filteredCloud = boost::make_shared<PointCloud>();
  filteredCloud->header = pclCloud_->header;
  filteredCloud->points.resize(pclCloud_->size());
  filteredCloud->clear();

  for (auto point_raw : pclCloud_->points)
  {
    double horizonDist = sqrt(point_raw.x * point_raw.x + point_raw.y * point_raw.y);

    if (horizonDist > _minRange && horizonDist < _maxRange)
    {
      auto point_new = point_raw;
      point_new.intensity = pointDistance(point_raw);
      filteredCloud->points.push_back(point_new);
    }
  }
  pclCloud_.swap(filteredCloud);
}

template <class T>
void PointcloudProcessor<T>::filterCloudByAngle(double _filterAngle)
{
  if (cloudIsEmpty())
    return;

  PointCloudPtr filteredCloud = boost::make_shared<PointCloud>();
  filteredCloud->header = pclCloud_->header;
  filteredCloud->points.resize(pclCloud_->size());
  filteredCloud->clear();

  for (auto point_raw : pclCloud_->points)
  {
    double horizonAngle = std::atan2(point_raw.y, point_raw.x);
    if (abs(horizonAngle) < M_PI - DEG2RAD(_filterAngle / 2))
    {
      auto point_new = point_raw;
      point_new.intensity = pointDistance(point_raw);
      filteredCloud->points.push_back(point_new);
    }
  }
  pclCloud_.swap(filteredCloud);
}

template <class T>
void PointcloudProcessor<T>::filterCloudByAxis(const std::string& _axis, double _minRange, double _maxRange,
                                               bool _negative)
{
  if (cloudIsEmpty())
    return;

  pcl::PassThrough<T> ps;
  ps.setInputCloud(pclCloud_);
  ps.setFilterFieldName(_axis);
  ps.setFilterLimits(_minRange, _maxRange);
  ps.setFilterLimitsNegative(_negative);
  ps.filter(*pclCloud_);
}

template <class T>
void PointcloudProcessor<T>::voxelDownsampling(double _voxelLeafSize)
{
  if (cloudIsEmpty())
    return;

  pcl::VoxelGrid<T> vox;
  vox.setInputCloud(pclCloud_);
  vox.setLeafSize(_voxelLeafSize, _voxelLeafSize, _voxelLeafSize);
  vox.filter(*pclCloud_);
}

template <class T>
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

template <class T>
bool PointcloudProcessor<T>::cloudIsEmpty()
{
  if (pclCloud_->empty())
  {
    ROS_WARN("point cloud empty. Skip passthrough filtering of the point cloud");
    return true;
  }

  return false;
}

}  // namespace ros

#endif  // ISR_CLOUD_PROCESSOR_ROS_H