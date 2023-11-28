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

#include <ros/ros.h>
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
  Subscriber(const std::string& topic, void (M::*fp)(T), M* obj);

  template <typename M>
  Subscriber(const std::string& topic, void (M::*fp)(T) const, M* obj);

  template <typename M>
  Subscriber(const std::string& topic, void (M::*fp)(const boost::shared_ptr<T const>&), M* obj);

  template <typename M>
  Subscriber(const std::string& topic, void (M::*fp)(const boost::shared_ptr<T const>&) const, M* obj);

  template <typename M>
  Subscriber(const std::string& topic, void (M::*fp)(T), const boost::shared_ptr<T>& obj);

  template <typename M>
  Subscriber(const std::string& topic, void (M::*fp)(T) const, const boost::shared_ptr<T>& obj);

  template <typename M>
  Subscriber(const std::string& topic, void (M::*fp)(const boost::shared_ptr<T const>&),
             const boost::shared_ptr<T>& obj);

  template <typename M>
  Subscriber(const std::string& topic, void (M::*fp)(const boost::shared_ptr<T const>&) const,
             const boost::shared_ptr<T>& obj);

  Subscriber(const std::string& topic, void (*fp)(T));

  Subscriber(const std::string& topic, void (*fp)(const boost::shared_ptr<T const>&));

  /// @brief
  /// @tparam M ROS Msg types to subscribe
  /// @param topic Topic to be subscribed
  /// @param queue_size Callback queue size
  /// @param fp Callback function
  /// @param obj Object to call function
  template <typename M>
  Subscriber(const std::string& topic, uint32_t queue_size, void (M::*fp)(const boost::shared_ptr<T const>&), M* obj);

  template <typename M>
  Subscriber(const std::string& topic, uint32_t queue_size, void (M::*fp)(T), M* obj);

  template <typename M>
  Subscriber(const std::string& topic, uint32_t queue_size, void (M::*fp)(T) const, M* obj);

  template <typename M>
  Subscriber(const std::string& topic, uint32_t queue_size, void (M::*fp)(const boost::shared_ptr<T const>&) const,
             M* obj);

  template <typename M>
  Subscriber(const std::string& topic, uint32_t queue_size, void (M::*fp)(T), const boost::shared_ptr<T>& obj);

  template <typename M>
  Subscriber(const std::string& topic, uint32_t queue_size, void (M::*fp)(T) const, const boost::shared_ptr<T>& obj);

  template <typename M>
  Subscriber(const std::string& topic, uint32_t queue_size, void (M::*fp)(const boost::shared_ptr<T const>&),
             const boost::shared_ptr<T>& obj);

  template <typename M>
  Subscriber(const std::string& topic, uint32_t queue_size, void (M::*fp)(const boost::shared_ptr<T const>&) const,
             const boost::shared_ptr<T>& obj);

  Subscriber(const std::string& topic, uint32_t queue_size, void (*fp)(T));

  Subscriber(const std::string& topic, uint32_t queue_size, void (*fp)(const boost::shared_ptr<T const>&));

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
Subscriber<T>::Subscriber(const std::string& topic, uint32_t queue_size, void (M::*fp)(T), M* obj)
  : topic_(topic), queue_size_(queue_size)
{
  sub_ = nh_.subscribe(topic_, queue_size_, fp, obj);
}

template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const std::string& topic, uint32_t queue_size, void (M::*fp)(T) const, M* obj)
  : topic_(topic), queue_size_(queue_size)
{
  sub_ = nh_.subscribe(topic_, queue_size_, fp, obj);
}

template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const std::string& topic, uint32_t queue_size,
                          void (M::*fp)(const boost::shared_ptr<T const>&), M* obj)
  : topic_(topic), queue_size_(queue_size)
{
  sub_ = nh_.subscribe(topic_, queue_size_, fp, obj);
}
template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const std::string& topic, uint32_t queue_size,
                          void (M::*fp)(const boost::shared_ptr<T const>&) const, M* obj)
  : topic_(topic), queue_size_(queue_size)
{
  sub_ = nh_.subscribe(topic_, queue_size_, fp, obj);
}

template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const std::string& topic, uint32_t queue_size, void (M::*fp)(T),
                          const boost::shared_ptr<T>& obj)
  : topic_(topic), queue_size_(queue_size)
{
  sub_ = nh_.subscribe(topic_, queue_size_, fp, obj);
}

template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const std::string& topic, uint32_t queue_size, void (M::*fp)(T) const,
                          const boost::shared_ptr<T>& obj)
  : topic_(topic), queue_size_(queue_size)
{
  sub_ = nh_.subscribe(topic_, queue_size_, fp, obj);
}

template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const std::string& topic, uint32_t queue_size,
                          void (M::*fp)(const boost::shared_ptr<T const>&), const boost::shared_ptr<T>& obj)
  : topic_(topic), queue_size_(queue_size)
{
  sub_ = nh_.subscribe(topic_, queue_size_, fp, obj);
}

template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const std::string& topic, uint32_t queue_size,
                          void (M::*fp)(const boost::shared_ptr<T const>&) const, const boost::shared_ptr<T>& obj)
  : topic_(topic), queue_size_(queue_size)
{
  sub_ = nh_.subscribe(topic_, queue_size_, fp, obj);
}

template <typename T>
Subscriber<T>::Subscriber(const std::string& topic, uint32_t queue_size, void (*fp)(T))
  : topic_(topic), queue_size_(queue_size)
{
  sub_ = nh_.subscribe(topic_, queue_size_, fp);
}

template <typename T>
Subscriber<T>::Subscriber(const std::string& topic, uint32_t queue_size, void (*fp)(const boost::shared_ptr<T const>&))
  : topic_(topic), queue_size_(queue_size)
{
  sub_ = nh_.subscribe(topic_, queue_size_, fp);
}

///////////////////////
template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const std::string& topic, void (M::*fp)(T), M* obj) : Subscriber(topic, 10, fp, obj)
{
}

template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const std::string& topic, void (M::*fp)(T) const, M* obj) : Subscriber(topic, 10, fp, obj)
{
}

template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const std::string& topic, void (M::*fp)(const boost::shared_ptr<T const>&), M* obj)
  : Subscriber(topic, 10, fp, obj)
{
}

template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const std::string& topic, void (M::*fp)(const boost::shared_ptr<T const>&) const, M* obj)
  : Subscriber(topic, 10, fp, obj)
{
}

template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const std::string& topic, void (M::*fp)(T), const boost::shared_ptr<T>& obj)
  : Subscriber(topic, 10, fp, obj)
{
}

template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const std::string& topic, void (M::*fp)(T) const, const boost::shared_ptr<T>& obj)
  : Subscriber(topic, 10, fp, obj)
{
}

template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const std::string& topic, void (M::*fp)(const boost::shared_ptr<T const>&),
                          const boost::shared_ptr<T>& obj)
  : Subscriber(topic, 10, fp, obj)
{
}

template <typename T>
template <typename M>
Subscriber<T>::Subscriber(const std::string& topic, void (M::*fp)(const boost::shared_ptr<T const>&) const,
                          const boost::shared_ptr<T>& obj)
  : Subscriber(topic, 10, fp, obj)
{
}

template <typename T>
Subscriber<T>::Subscriber(const std::string& topic, void (*fp)(T)) : Subscriber(topic, 10, fp)
{
}

template <typename T>
Subscriber<T>::Subscriber(const std::string& topic, void (*fp)(const boost::shared_ptr<T const>&))
  : Subscriber(topic, 10, fp)
{
}

}  // namespace roscpp
#endif  // ISR_ROSCPP_CORE_SUBSCRIBER_H