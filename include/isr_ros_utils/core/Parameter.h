/*
 * parameter.h
 *
 *  Created on: Sep 20, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ISR_ROSCPP_CORE_PARAMETER_H
#define ISR_ROSCPP_CORE_PARAMETER_H

#include <ros/param.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

namespace isr::roscpp
{
template <typename T>
class Parameter
{
public:
  /// @brief Constructor
  Parameter(){};

  /// @brief Constructor
  /// @param val Default value to be set
  Parameter(const T& val);

  /// @brief
  /// @param key The key to be searched on the parameter server
  /// @param val Default value to be set if the key doesn't exist on the parameter server
  Parameter(const std::string& key, const T& val);

  /// @brief
  /// @return Returns parameter value as const
  const T& value() const;

  /// @brief
  /// @return Returns parameter value as non-const
  T& get();

  /// @brief
  /// @param val The value to be set
  void set(const T& val);

  /// @brief
  /// @param param_name The key to be searched on the parameter server
  /// @return True if the key is searched on the parameter server. Otherwise False
  bool readParameter(const std::string& param_name);

  /// @brief
  /// @param param_name The key to be searched on the parameter server
  /// @param default_val Value to use if the server doesn't have the key
  /// @return True if the key is searched on the parameter server. Otherwise False
  bool readParameter(const std::string& param_name, const T& default_val);

private:
  T val_{};
};

template <typename T>
Parameter<T>::Parameter(const T& val) : val_(val)
{
}

template <typename T>
Parameter<T>::Parameter(const std::string& key, const T& val)
{
  readParameter(key, val);
}

template <typename T>
inline const T& Parameter<T>::value() const
{
  return val_;
}

template <typename T>
inline T& Parameter<T>::get()
{
  return val_;
}

template <typename T>
inline void Parameter<T>::set(const T& val)
{
  val_ = val;
}

template <typename T>
inline bool Parameter<T>::readParameter(const std::string& param_name)
{
  bool success = ros::param::get(param_name, val_);
  ROS_ERROR_STREAM_COND(!success, "Could not read parameter " << ros::names::resolve(param_name));

  return success;
}

template <typename T>
inline bool Parameter<T>::readParameter(const std::string& param_name, const T& val_default)
{
  bool success = ros::param::param(param_name, val_, val_default);
  ROS_WARN_STREAM_COND(!success,
                       "Could not read parameter " << ros::names::resolve(param_name) << ". Use default Value");

  return success;
}

}  // namespace isr::roscpp
#endif  // ISR_ROSCPP_CORE_PARAMETER_H