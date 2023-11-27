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
#include "isr_ros_utils/core/Parameter.h"

namespace roscpp
{
template <typename T>
class Publisher
{
public:
  /// @brief Constructor: publisher queue size is set to 10 in default
  /// @param topic Topic to be published
  Publisher(const std::string& topic);

  /// @brief Constructor
  /// @param topic Topic to be published
  /// @param queue_size Msg queue size
  Publisher(const std::string& topic, uint32_t queue_size);

  /// @brief Constructor: publisher queue size is set to 10 in default
  /// @param nh Namespace to be added to topic
  /// @param topic Topic to be published
  Publisher(const ros::NodeHandle& nh, const std::string& topic);

  /// @brief Constructor: use this for setting new namespace in topic
  /// @param nh nodeHandle that has new namespace
  /// @param topic Topic to be published
  Publisher(const ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size);

  /// @brief
  /// @param topic_param Parameterized topic to be published using ROS parameter server
  Publisher(const roscpp::Parameter<std::string>& topic_param);

  /// @brief
  /// @param topic_param Parameterized topic to be published using ROS parameter server
  Publisher(const roscpp::Parameter<std::string>& topic_param, uint32_t queue_size);

  /// @brief
  /// @param nh nodeHandle that has new namespace
  /// @param topic_param Parameterized topic to be published using ROS parameter server
  Publisher(const ros::NodeHandle& nh, const roscpp::Parameter<std::string>& topic_param);

  /// @brief
  /// @param nh nodeHandle that has new namespace
  /// @param topic_param Parameterized topic to be published using ROS parameter server
  /// @param queue_size Msg queue size
  Publisher(const ros::NodeHandle& nh, const roscpp::Parameter<std::string>& topic_param, uint32_t queue_size);

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

  /// @brief
  /// @return Get publisher msg queue size
  int getQueueSize() const;

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  roscpp::Parameter<std::string> topic_;
  roscpp::Parameter<int> queue_size_;
};

template <typename T>
Publisher<T>::Publisher(const ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size)
  : nh_(nh), topic_(topic), queue_size_(queue_size)
{
  pub_ = nh_.advertise<T>(topic_.value(), queue_size_.value());
}

template <typename T>
Publisher<T>::Publisher(const std::string& topic) : Publisher(ros::NodeHandle("~"), topic, 10)
{
}

template <typename T>
Publisher<T>::Publisher(const std::string& topic, uint32_t queue_size)
  : Publisher(ros::NodeHandle("~"), topic, queue_size)
{
}

template <typename T>
Publisher<T>::Publisher(const ros::NodeHandle& nh, const std::string& topic) : Publisher(nh, topic, 10)
{
}

template <typename T>
Publisher<T>::Publisher(const roscpp::Parameter<std::string>& topic_param)
  : Publisher(ros::NodeHandle("~"), topic_param.value(), 10)
{
}

template <typename T>
Publisher<T>::Publisher(const roscpp::Parameter<std::string>& topic_param, uint32_t queue_size)
  : Publisher(ros::NodeHandle("~"), topic_param.value(), queue_size)
{
}

template <typename T>
Publisher<T>::Publisher(const ros::NodeHandle& nh, const roscpp::Parameter<std::string>& topic_param)
  : Publisher(nh, topic_param.value(), 10)
{
}

template <typename T>
Publisher<T>::Publisher(const ros::NodeHandle& nh, const roscpp::Parameter<std::string>& topic_param,
                        uint32_t queue_size)
  : Publisher(nh, topic_param.value(), queue_size)
{
}

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

}  // namespace roscpp

#endif  // ISR_ROSCPP_CORE_PUBLISHER_H