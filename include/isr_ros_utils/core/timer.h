/*
 * timer.h
 *
 *  Created on: Sep 20, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ISR_ROSCPP_CORE_TIMER_H
#define ISR_ROSCPP_CORE_TIMER_H

#include <ros/timer.h>
#include "isr_ros_utils/core/parameter.h"

namespace isr::roscpp
{
class Timer
{
public:
  template <typename T>
  explicit Timer(const ros::Duration& duration, void (T::*fp)(const ros::TimerEvent&), T* obj, bool autostart = false,
        bool oneshot = false)
    : duration_sec_(duration.toSec())
  {
    timer_ = nh_.createTimer(duration, fp, obj, oneshot, autostart);
  }

  template <typename T>
  explicit Timer(const roscpp::Parameter<double>& duration_param, void (T::*fp)(const ros::TimerEvent&), T* obj,
        bool autostart = false, bool oneshot = false)
    : duration_sec_(duration_param.value())
  {
    timer_ = nh_.createTimer(ros::Duration(duration_sec_), fp, obj, oneshot, autostart);
  }

  // Timer(ros::Duration default_duration) : duration_(default_duration.toSec())
  // {
  // }

  // Timer(const ros::NodeHandle& nh, double default_duration) : nh_(nh), duration_(default_duration)
  // {
  // }

  // Timer(const ros::NodeHandle& nh, ros::Duration default_duration) : nh_(nh), duration_(default_duration.toSec())
  // {
  // }

  void start()
  {
    timer_.start();
  }

  void stop()
  {
    timer_.stop();
  }

  double getDuration() const
  {
    return duration_sec_;
  }

  bool hasStarted() const
  {
    return timer_.hasStarted();
  }

private:
  ros::NodeHandle nh_;
  ros::Timer timer_;
  double duration_sec_;
};

}  // namespace isr::roscpp

#endif  // ISR_ROSCPP_CORE_TIMER_H