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
#include <ros/node_handle.h>

#include "isr_ros_utils/core/Parameter.h"

namespace roscpp
{
class Timer
{
public:
  /// @brief
  /// @tparam T
  /// @param duration
  /// @param fp
  /// @param obj
  /// @param autostart
  /// @param oneshot
  template <typename T>
  explicit Timer(const ros::Duration& duration, void (T::*fp)(const ros::TimerEvent&), T* obj, bool autostart = false,
                 bool oneshot = false);

  /// @brief
  /// @tparam T
  /// @param duration_param
  /// @param fp
  /// @param obj
  /// @param autostart
  /// @param oneshot
  template <typename T>
  explicit Timer(const roscpp::Parameter<double>& duration_param, void (T::*fp)(const ros::TimerEvent&), T* obj,
                 bool autostart = false, bool oneshot = false);

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

  bool isRunning() const
  {
    return timer_.hasStarted();
  }

private:
  ros::NodeHandle nh_private_{ "~" };
  ros::Timer timer_;
  double duration_sec_;
};

template <typename T>
Timer::Timer(const ros::Duration& duration, void (T::*fp)(const ros::TimerEvent&), T* obj, bool autostart, bool oneshot)
  : duration_sec_(duration.toSec())
{
  timer_ = nh_private_.createTimer(duration, fp, obj, oneshot, autostart);
}

template <typename T>
Timer::Timer(const roscpp::Parameter<double>& duration_param, void (T::*fp)(const ros::TimerEvent&), T* obj,
             bool autostart, bool oneshot)
  : Timer(ros::Duration(duration_param.value()), fp, obj, autostart, oneshot)
{
}

}  // namespace roscpp

#endif  // ISR_ROSCPP_CORE_TIMER_H