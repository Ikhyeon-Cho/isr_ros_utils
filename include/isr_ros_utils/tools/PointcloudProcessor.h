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
  using PointCloud = pcl::PointCloud<T>;
  using PointCloudPtr = pcl::PointCloud<T>::Ptr;
  // typedef pcl::PointCloud<T> PointCloud;
  // typedef typename pcl::PointCloud<T>::Ptr PointCloudPtr;

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
  ros::TransformHandler transform_handler_;
  PointCloudPtr pclCloud_;
};

}  // namespace ros

#endif  // ISR_CLOUD_PROCESSOR_ROS_H