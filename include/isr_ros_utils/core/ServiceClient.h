/*
 * ServiceClient.h
 *
 *  Created on: Sep 20, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ISR_ROS_SERVICE_CLIENT
#define ISR_ROS_SERVICE_CLIENT

#include <ros/param.h>
#include <std_srvs/Empty.h>

namespace roscpp
{
class ServiceClient
{
private:
  std::string service_;

public:
  ServiceClient(){};
  ServiceClient(const std::string& _default_val) : service_(_default_val)
  {
  }

  /*
   * \param _param_name The key to be searched on the parameter server.
   * **/
  void readParameter(const std::string& _param_name)
  {
    bool readParameter = ros::param::get(_param_name, service_);
    ROS_ERROR_STREAM_COND(!readParameter, "Could not read service parameter " << _param_name);
  }

  /*
   * \param _param_name The key to be searched on the parameter server.
   * \param _default_val Value to use if the server doesn't contain this
   * **/
  void readParameter(const std::string& _param_name, const std::string& _default_val)
  {
    bool readParameter = ros::param::param(_param_name, service_, _default_val);
    ROS_WARN_STREAM_COND(!readParameter, "Could not read service parameter " << _param_name << ". Use default Value");
  }
};
}  // namespace roscpp

#endif