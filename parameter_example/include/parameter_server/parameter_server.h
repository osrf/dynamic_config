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
#include "parameter_example/OnUpdate.h"
#include "parameter_example/AddOnChange.h"
#include "parameter_example/RemoveOnChange.h"
#include "parameter_example/OnChange.h"

namespace parameter_server {

class ParameterServer
{
  typedef std::vector<uint8_t> Data;
  typedef std::map<std::string, Data> DataMap;
  typedef std::set<std::string> ParameterListeners;
  typedef std::map<std::string, ParameterListeners> Listeners;
public:
  ParameterServer(ros::NodeHandle n);

private:

  bool set_handler(parameter_example::Set::Request  &req,
                   parameter_example::Set::Response &res);

  bool get_handler(parameter_example::Get::Request  &req,
                   parameter_example::Get::Response &res);

  bool add_on_update_handler(parameter_example::AddOnUpdate::Request  &req,
                             parameter_example::AddOnUpdate::Response &res);

  bool remove_on_update_handler(parameter_example::RemoveOnUpdate::Request  &req,
                                parameter_example::RemoveOnUpdate::Response &res);

  bool add_on_change_handler(parameter_example::AddOnChange::Request  &req,
                             parameter_example::AddOnChange::Response &res);

  bool remove_on_change_handler(parameter_example::AddOnChange::Request  &req,
                                parameter_example::AddOnChange::Response &res);

  // Call all on change listeners except the listener which matches no_call. This is to avoid
  // calling the same node is calling you. If the caller node isn't multithread it will block.
  // By default returns true
  bool accept_change(const std::string& name, const Data& data, const std::string& no_call);

  // Notify all on update listeners except the listener which matches no_call.
  void notify_update(const std::string& name, const Data& data, const std::string& no_call);

  std::vector<ros::ServiceServer> serviceServers;
  DataMap dataMap;
  Listeners onUpdateListeners;
  Listeners onChangeListeners;
};

}