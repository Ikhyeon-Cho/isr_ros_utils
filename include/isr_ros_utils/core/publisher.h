/*
 * publisher.h
 *
 *  Created on: Sep 20, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ISR_ROSCPP_CORE_PUBLISHER_H
#define ISR_ROSCPP_CORE_PUBLISHER_H

#include <ros/publisher.h>
#include <isr_ros_utils/core/parameter.h>

namespace isr::roscpp
{
template <typename T>
class Publisher
{
public:
  /// @brief Constructor: publisher queue size is set to 10 in default
  /// @param topic Topic to be published
  Publisher(const std::string& topic) : nh_("~"), topic_(topic)
  {
    pub_ = nh_.advertise<T>(topic_.value(), queue_size_.value());
  }

  /// @brief Constructor
  /// @param topic Topic to be published
  /// @param queue_size Msg queue size
  Publisher(const std::string& topic, uint32_t queue_size) : nh_("~"), topic_(topic), queue_size_(queue_size)
  {
    pub_ = nh_.advertise<T>(topic_.value(), queue_size_.value());
  }

  /// @brief Constructor: use this for setting new namespace in topic
  /// @param nh nodeHandle that has new namespace
  /// @param topic Topic to be published
  Publisher(const ros::NodeHandle& nh, const std::string& topic) : nh_(nh), topic_(topic)
  {
    pub_ = nh_.advertise<T>(topic_.value(), queue_size_.value());
  }

  /// @brief
  /// @param key The key to be searched on the parameter server
  /// @return True if the key is searched on the parameter server. Otherwise False
  bool readParameter(const std::string& key);

  /// @brief
  /// @param key The key to be searched on the parameter server
  /// @param default_topic Default topic to use if the server doesn't have topic parameters
  /// @return True if the key is searched on the parameter server. Otherwise False
  bool readParameter(const std::string& key, const std::string& default_topic);

  /// @brief
  /// @param msg Msg to be published
  void publish(const T& msg);

  /// @brief
  /// @return Get topic name
  std::string getTopic() const;

  int getQueueSize() const;

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  roscpp::Parameter<std::string> topic_;
  roscpp::Parameter<int> queue_size_{ 10 };
};

template <typename T>
inline bool Publisher<T>::readParameter(const std::string& param_name, const std::string& default_topic)
{
  bool get_param = topic_.readParameter(param_name, default_topic);
  pub_ = nh_.advertise<T>(topic_.value(), queue_size_.value());
  return get_param;
}

template <typename T>
inline bool Publisher<T>::readParameter(const std::string& param_name)
{
  bool get_param = topic_.readParameter(param_name);
  pub_ = nh_.advertise<T>(topic_.value(), queue_size_.value());
  return get_param;
}

template <typename T>
inline void Publisher<T>::publish(const T& msg)
{
  pub_.publish(msg);
}

template <typename T>
inline std::string Publisher<T>::getTopic() const
{
  return pub_.getTopic();
}

template <typename T>
inline int Publisher<T>::getQueueSize() const
{
  return queue_size_.value();
}

}  // namespace isr::roscpp

#endif  // ISR_ROSCPP_CORE_PUBLISHER_H