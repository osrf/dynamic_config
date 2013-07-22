/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation, Inc. nor
 *     the names of its contributors may be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"

#include "parameter_example/Set.h"
#include "parameter_example/Get.h"
#include "parameter_example/AddOnUpdate.h"
#include "parameter_example/RemoveOnUpdate.h"
#include "parameter_example/AddOnChange.h"
#include "parameter_example/RemoveOnChange.h"

namespace ros_parameter {

  namespace {

    template <class Srv>
    bool call_service(const std::string& srv_name, Srv& srv)
    {
      bool successful = ros::service::call(srv_name, srv);
      if (!successful)
        ROS_ERROR_STREAM("Service " << srv_name << " is not available");
      return successful;
    }

    template <class Msg>
    Msg make_name_service(const std::string& name, const std::string& service)
    {
      Msg msg;
      msg.request.name = name;
      msg.request.service = service;
      return msg;
    }

} // anonymous

template <typename Data>
bool set_parameter(const std::string& name, const Data& data, const std::string& no_call)
{
  parameter_example::Set srv;
  srv.request.name = name;
  srv.request.data = data;
  srv.request.no_call = no_call;

  return call_service("/parameters_server/set", srv) && srv.response.accepted; 
}

template <typename Data>
bool get_parameter(const std::string& name,
                   const Data& default_data,
                   Data& data)
{
  parameter_example::Get srv;
  srv.request.name = name;
  srv.request.data = default_data;

  bool successful = call_service("/parameters_server/get", srv);
  if (successful)
    data = srv.response.data;
  return successful;
}

bool add_on_update_parameter_listener(const std::string& name,
                                      const std::string& service)
{
  parameter_example::AddOnUpdate srv = make_name_service<parameter_example::AddOnUpdate>(name, service);
  return call_service("/parameters_server/add_on_update", srv);
}

bool remove_on_update_parameter_listener(const std::string& name,
                                      const std::string& service)
{
  parameter_example::RemoveOnUpdate srv = make_name_service<parameter_example::RemoveOnUpdate>(name, service);
  return call_service("/parameters_server/remove_on_update", srv);
}

bool add_on_change_parameter_listener(const std::string& name,
                                      const std::string& service)
{
  parameter_example::AddOnChange srv = make_name_service<parameter_example::AddOnChange>(name, service);
  return call_service("/parameters_server/add_on_change", srv);
}

bool remove_on_change_parameter_listener(const std::string& name,
                                         const std::string& service)
{
  parameter_example::RemoveOnChange srv = make_name_service<parameter_example::RemoveOnChange>(name, service);
  return call_service("/parameters_server/remove_on_change", srv);
}

} // ros_parameter
