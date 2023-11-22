/*
 * subscriber.h
 *
 *  Created on: Sep 20, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ISR_ROSCPP_CORE_SUBSCRIBER_H
#define ISR_ROSCPP_CORE_SUBSCRIBER_H

#include <isr_ros_utils/core/parameter.h>

namespace isr::roscpp
{
template <typename T>
class Subscriber
{
public:
  /// @brief
  /// @tparam M ROS Msg types to subscribe
  /// @param topic Topic to be subscribed
  /// @param fp Callback function
  /// @param obj Object to call function
  template <typename M>
  Subscriber(const std::string& topic, void (M::*fp)(const boost::shared_ptr<T const>&), M* obj) : topic_(topic)
  {
    sub_ = nh_.subscribe<T>(topic_.value(), queue_size_.value(), fp, obj);
  }

  /// @brief
  /// @tparam M ROS Msg types to subscribe
  /// @param topic Topic to be subscribed
  /// @param queue_size Callback queue size
  /// @param fp Callback function
  /// @param obj Object to call function
  template <typename M>
  Subscriber(const std::string& topic, uint32_t queue_size, void (M::*fp)(const boost::shared_ptr<T const>&), M* obj)
    : topic_(topic), queue_size_(queue_size)
  {
    sub_ = nh_.subscribe<T>(topic_.value(), queue_size_.value(), fp, obj);
  }

  /// @brief
  /// @tparam M ROS Msg types to subscribe
  /// @param nh nodeHandle that has new namespace
  /// @param topic Topic to be subscribed
  /// @param fp Callback function
  /// @param obj Object to call function
  template <typename M>
  Subscriber(const ros::NodeHandle& nh, const std::string& topic, void (M::*fp)(const boost::shared_ptr<T const>&),
             M* obj)
    : nh_(nh), topic_(topic)
  {
    sub_ = nh_.subscribe<T>(topic_.value(), queue_size_.value(), fp, obj);
  }

  template <typename M>
  Subscriber(const roscpp::Parameter<std::string>& topic_param, void (M::*fp)(const boost::shared_ptr<T const>&), M* obj)
    : topic_(topic_param.value())
  {
    sub_ = nh_.subscribe<T>(topic_.value(), queue_size_.value(), fp, obj);
  }

  /// @brief
  /// @return Topic that subscribes
  std::string getTopic() const
  {
    return sub_.getTopic();
  }

  /// @brief
  /// @return Callback queue size
  int getQueueSize() const
  {
    return queue_size_.value();
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  Parameter<std::string> topic_;
  Parameter<int> queue_size_{ 10 };
};

}  // namespace isr::roscpp
#endif  // ISR_ROSCPP_CORE_SUBSCRIBER_H