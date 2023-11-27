/*
 * DynamicReconfigureServer.h
 *
 *  Created on: Oct 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ROSCPP_DYNAMIC_RECONFIGURE_H
#define ROSCPP_DYNAMIC_RECONFIGURE_H

#include "isr_ros_utils/core/Parameter.h"
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

namespace roscpp
{
class DynamicReconfigureServer
{
public:
  /// @brief By registering variables (mostly the parameters) to the server,
  /// they become dynamic and adjustable during node operation
  /// @param nh Set server namespace via ros nodehandle.
  DynamicReconfigureServer(const ros::NodeHandle& nh) : ddr_(nh)
  {
  }

  /// @brief
  /// @tparam T Int, Bool, Double, String types are available. Float is not supported in dynamic_reconfigure
  /// @param name Names are shown in dynamic reconfigure panel (rqt)
  /// @param variable Variable to be registered to Dynamic reconfigure server
  /// @param min The minimum value that the variable should have
  /// @param max The maximum value that the variable should have
  template <typename T>
  void registerVariable(const std::string& name, T& variable, T min = getMin<T>(), T max = getMax<T>())
  {
    ddr_.registerVariable<T>(name, &variable, "", min, max);
  }

  /// @brief
  /// @param name Names are shown in dynamic reconfigure panel (rqt)
  /// @param variable Variable to be registered to Dynamic reconfigure server
  /// @param callback Callback to be called when the variable is changed
  /// @param obj Callback owner
  /// @param min The minimum value that the variable should have
  /// @param max The maximum value that the variable should have
  template <typename T, typename M>
  void registerVariable(const std::string& name, T& variable, void (M::*callback)(T), M* obj, T min = getMin<T>(),
                        T max = getMax<T>())
  {
    ddr_.registerVariable<T>(name, &variable, boost::bind(callback, obj, boost::placeholders::_1), "", min, max);
  }

  /// @brief Activate server. Call this only once per node
  void activate()
  {
    ddr_.publishServicesTopics();
  }

private:
  ddynamic_reconfigure::DDynamicReconfigure ddr_;
};

}  // namespace roscpp

#endif  // ROSCPP_DYNAMIC_RECONFIGURE_H