/*
 * node.h
 *
 *  Created on: Sep 20, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ISR_ROSCPP_CORE_NODE_H
#define ISR_ROSCPP_CORE_NODE_H

// #include <ros/ros.h>
#include <isr_ros_utils/core/publisher.h>
#include <isr_ros_utils/core/subscriber.h>
#include <isr_ros_utils/core/timer.h>
#include <isr_ros_utils/core/parameter.h>
#include <isr_ros_utils/tools/transform_handler.h>

namespace isr::roscpp
{
class Node
{
public:
  TransformHandler transform_handler{};

  std::string map_frame_;
  std::string base_frame_;
  std::string odom_frame_;
  std::string lidar_frame_;

private:
  ros::NodeHandle nh_;  // need this to start and shutdown the node lifetime

public:
  Node() : nh_("~")
  {
    ros::param::param<std::string>("/FrameIds/map", map_frame_, "map");
    ros::param::param<std::string>("/FrameIds/base", base_frame_, "base_link");
    ros::param::param<std::string>("/FrameIds/odom", odom_frame_, "odom");
    ros::param::param<std::string>("/FrameIds/lidar", lidar_frame_, "lidar");
  }
  Node(const ros::NodeHandle& nh)
  {
    ros::param::param<std::string>("/FrameIds/map", map_frame_, "map");
    ros::param::param<std::string>("/FrameIds/base", base_frame_, "base_link");
    ros::param::param<std::string>("/FrameIds/odom", odom_frame_, "odom");
    ros::param::param<std::string>("/FrameIds/lidar", lidar_frame_, "lidar");
  }
};

}  // namespace isr::roscpp

#endif  // ISR_ROSCPP_CORE_NODE_H