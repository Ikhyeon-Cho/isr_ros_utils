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

#include <ros/subscriber.h>
#include "isr_ros_utils/core/Parameter.h"

namespace roscpp
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
  Subscriber(const std::string& topic, void (M::*fp)(const boost::shared_ptr<T const>&), M* obj);

  /// @brief
  /// @tparam M ROS Msg types to subscribe
  /// @param topic Topic to be subscribed
  /// @param queue_size Callback queue size
  /// @param fp Callback function
  /// @param obj Object to call function
  template <typename M>
  Subscriber(const std::string& topic, uint32_t queue_size, void (M::*fp)(const boost::shared_ptr<T const>&), M* obj);

  /// @brief
  /// @tparam M ROS Msg types to subscribe
  /// @param nh nodeHandle that has new namespace
  /// @param topic Topic to be subscribed
  /// @param fp Callback function
  /// @param obj Object to call function
  template <typename M>
  Subscriber(const ros::NodeHandle& nh, const std::string& topic, void (M::*fp)(const boost::shared_ptr<T const>&),
             M* obj);

  /// @brief
  /// @tparam M
  /// @param nh
  /// @param topic
  /// @param queue_size
  /// @param fp
  /// @param obj
  template <typename M>
  Subscriber(const ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size,
             void (M::*fp)(const boost::shared_ptr<T const>&), M* obj);

  /// @brief
  /// @tparam M
  /// @param topic_param
  /// @param fp
  /// @param obj
  template <typename M>
  Subscriber(const roscpp::Parameter<std::string>& topic_param, void (M::*fp)(const boost::shared_ptr<T const>&),
             M* obj);

  /// @brief
  /// @tparam M
  /// @param topic_param
  /// @param queue_size
  /// @param fp
  /// @param obj
  template <typename M>
  Subscriber(const roscpp::Parameter<std::string>& topic_param, uint32_t queue_size,
                            void (M::*fp)(const boost::shared_ptr<T const>&), M* obj);

  /// @brief
  /// @tparam M
  /// @param nh
  /// @param topic_param
  /// @param fp
  /// @param obj
  template <typename M>
  Subscriber(const ros::NodeHandle& nh, const roscpp::Parameter<std::string>& topic_param,
             void (M::*fp)(const boost::shared_ptr<T const>&), M* obj);

  /// @brief
  /// @tparam M
  /// @param nh
  /// @param topic_param
  /// @param queue_size
  /// @param fp
  /// @param obj
  template <typename M>
  Subscriber(const ros::NodeHandle& nh, const roscpp::Parameter<std::string>& topic_param, uint32_t queue_size,
             void (M::*fp)(const boost::shared_ptr<T const>&), M* obj);

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
    return queue_size_;
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::string topic_;
  uint32_t queue_size_;
};

template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size,
                          void (M::*fp)(const boost::shared_ptr<T const>&), M* obj)
  : nh_(nh), topic_(topic), queue_size_(queue_size)
{
  sub_ = nh_.subscribe<T>(topic_, queue_size_, fp, obj);
}

template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const std::string& topic, void (M::*fp)(const boost::shared_ptr<T const>&), M* obj)
  : Subscriber(nh_, topic, 10, fp, obj)
{
}

template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const std::string& topic, uint32_t queue_size,
                          void (M::*fp)(const boost::shared_ptr<T const>&), M* obj)
  : Subscriber(nh_, topic, queue_size, fp, obj)
{
}

template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const ros::NodeHandle& nh, const std::string& topic,
                          void (M::*fp)(const boost::shared_ptr<T const>&), M* obj)
  : Subscriber(nh_, topic, 10, fp, obj)
{
}

template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const roscpp::Parameter<std::string>& topic_param,
                          void (M::*fp)(const boost::shared_ptr<T const>&), M* obj)
  : Subscriber(nh_, topic_param.value(), 10, fp, obj)
{
}

template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const roscpp::Parameter<std::string>& topic_param, uint32_t queue_size,
                          void (M::*fp)(const boost::shared_ptr<T const>&), M* obj)
  : Subscriber(nh_, topic_param.value(), queue_size, fp, obj)
{
}

template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const ros::NodeHandle& nh, const roscpp::Parameter<std::string>& topic_param,
                          void (M::*fp)(const boost::shared_ptr<T const>&), M* obj)
  : Subscriber(nh, topic_param.value(), 10, fp, obj)
{
}

template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const ros::NodeHandle& nh, const roscpp::Parameter<std::string>& topic_param,
                          uint32_t queue_size, void (M::*fp)(const boost::shared_ptr<T const>&), M* obj)
  : Subscriber(nh, topic_param.value(), queue_size, fp, obj)
{
}

}  // namespace roscpp
#endif  // ISR_ROSCPP_CORE_SUBSCRIBER_H