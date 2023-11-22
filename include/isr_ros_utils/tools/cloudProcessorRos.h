/*
 * cloud_processor_ros.h
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
#include "transform_handler.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// Use the Velodyne point format as a common representation

namespace isr
{
template <class T>
class CloudProcessorROS
{
  typedef pcl::PointCloud<T> pCloud;
  typedef typename pcl::PointCloud<T>::Ptr pCloudPtr;

private:
  roscpp::TransformHandler transform_handler_;

  pCloudPtr pclCloud_;

public:
  CloudProcessorROS();

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
};

template <class T>
CloudProcessorROS<T>::CloudProcessorROS()
{
  pclCloud_ = boost::make_shared<pCloud>();
}

template <class T>
void CloudProcessorROS<T>::setPointCloud(const sensor_msgs::PointCloud2& _msg)
{
  pclCloud_->clear();
  pcl::fromROSMsg(_msg, *pclCloud_);
}

template <class T>
void CloudProcessorROS<T>::getPointCloud(sensor_msgs::PointCloud2& _cloud)
{
  pcl::toROSMsg(*pclCloud_, _cloud);
}

template <class T>
sensor_msgs::PointCloud2Ptr CloudProcessorROS<T>::getPointCloud()
{
  sensor_msgs::PointCloud2Ptr cloudPtr = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(*pclCloud_, *cloudPtr);
  return cloudPtr;
}

template <class T>
void CloudProcessorROS<T>::clearPointCloud()
{
  pclCloud_->clear();
}

template <class T>
bool CloudProcessorROS<T>::transformPointCloudTo(const std::string& target_frame)
{
  std::string source_frame = pclCloud_->header.frame_id;
  if (source_frame.empty())
  {
    ROS_ERROR_STREAM("cloud cannot be transformed because it has no frame id");
    return false;
  }

  if (source_frame == target_frame)
    return true;

  geometry_msgs::TransformStamped transform_stamped;
  ros::Time timestamp = pcl_conversions::fromPCL(pclCloud_->header.stamp);
  if (!transform_handler_.getTransform(transform_stamped, target_frame, source_frame, timestamp))
    return false;

  Eigen::Affine3d transform_eigen = transform_handler_.toEigen(transform_stamped.transform);

  pCloudPtr transformedCloud = boost::make_shared<pCloud>();
  transformedCloud->header = pclCloud_->header;

  pcl::transformPointCloud(*pclCloud_, *transformedCloud, transform_eigen);
  pclCloud_.swap(transformedCloud);
  pclCloud_->header.frame_id = target_frame;

  return true;
}

template <class T>
void CloudProcessorROS<T>::filterCloudByRange(double _minRange, double _maxRange)
{
  if (cloudIsEmpty())
    return;

  pCloudPtr filteredCloud = boost::make_shared<pCloud>();
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
void CloudProcessorROS<T>::filterCloudByAngle(double _filterAngle)
{
  if (cloudIsEmpty())
    return;

  pCloudPtr filteredCloud = boost::make_shared<pCloud>();
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
void CloudProcessorROS<T>::filterCloudByAxis(const std::string& _axis, double _minRange, double _maxRange,
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
void CloudProcessorROS<T>::voxelDownsampling(double _voxelLeafSize)
{
  if (cloudIsEmpty())
    return;

  pcl::VoxelGrid<T> vox;
  vox.setInputCloud(pclCloud_);
  vox.setLeafSize(_voxelLeafSize, _voxelLeafSize, _voxelLeafSize);
  vox.filter(*pclCloud_);
}

template <class T>
void CloudProcessorROS<T>::filterRingOfIndex(double _ringIndex, bool _negative)
{
  pCloudPtr filteredCloud = boost::make_shared<pCloud>();
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
bool CloudProcessorROS<T>::cloudIsEmpty()
{
  if (pclCloud_->empty())
  {
    ROS_WARN("point cloud empty. Skip passthrough filtering of the point cloud");
    return true;
  }

  return false;
}
}  // namespace isr

#endif  // ISR_CLOUD_PROCESSOR_ROS_H