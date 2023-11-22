/*
 * dynamic_reconfigure_handler.h
 *
 *  Created on: Oct 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ISR_ROSCPP_TOOLS_DYNAMIC_RECONFIGURE_HANDLER_H
#define ISR_ROSCPP_TOOLS_DYNAMIC_RECONFIGURE_HANDLER_H

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

namespace isr::roscpp
{
class DynamicReconfigureHandler
{
public:
  DynamicReconfigureHandler(const ros::NodeHandle& nh) : ddr_(nh)
  {
  }

  template <class T>
  void makeDynamic(T& variable, const std::string& name, T min = getMin<T>(), T max = getMax<T>())
  {
    ddr_.registerVariable<T>(name, &variable, "", min, max);
  }

  template <class T, class M>
  void makeDynamic(T& variable, const std::string& name, void (M::*_callback)(T), M* _obj, T min = getMin<T>(),
                   T max = getMax<T>())
  {
    ddr_.registerVariable<T>(name, &variable, boost::bind(_callback, _obj, boost::placeholders::_1), "", min, max);
  }

  void registration()
  {
    ddr_.publishServicesTopics();
  }

private:
  ddynamic_reconfigure::DDynamicReconfigure ddr_;
};
}  // namespace isr::roscpp

#endif  // ISR_ROSCPP_TOOLS_DYNAMIC_RECONFIGURE_HANDLER_H