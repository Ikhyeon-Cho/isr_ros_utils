/*
 * velodyne_point.h
 *
 *  Created on: Dec 7, 2021
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef VELODYNE_POINT_H
#define VELODYNE_POINT_H

#include <pcl/point_types.h>
#include <pcl/point_traits.h>

// Define the custom Velodyne point type
struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D; // Adds x,y,z coordinates and padding
    PCL_ADD_INTENSITY;
    std::uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Make sure our new allocators are aligned
};

// Register the custom point type with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(float, time, time))

#endif // VELODYNE_POINT_H
