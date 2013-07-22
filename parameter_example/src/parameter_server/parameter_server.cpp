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

#include "parameter_server/parameter_server.h"

namespace parameter_server {

ParameterServer::ParameterServer(ros::NodeHandle n)
: serviceServers(6)
{
  serviceServers.push_back( n.advertiseService("set", &ParameterServer::set_handler, this) );
  serviceServers.push_back( n.advertiseService("get", &ParameterServer::get_handler, this) );
  serviceServers.push_back( n.advertiseService("add_on_update", &ParameterServer::add_on_update_handler, this) );
  serviceServers.push_back( n.advertiseService("remove_on_update", &ParameterServer::remove_on_update_handler, this) );
  serviceServers.push_back( n.advertiseService("add_on_change", &ParameterServer::add_on_change_handler, this) );
  serviceServers.push_back( n.advertiseService("remove_on_change", &ParameterServer::remove_on_change_handler, this) );
}

bool 
ParameterServer::set_handler(parameter_example::Set::Request  &req,
                             parameter_example::Set::Response &res)
{
  res.accepted = accept_change(req.name, req.data, req.no_call);
  if (res.accepted) {
    dataMap[req.name] = req.data;
    res.accepted = true;
    notify_update(req.name, req.data, req.no_call);
  }
  return true;
}

bool 
ParameterServer::accept_change(const std::string& name, const Data& data, const std::string& no_call)
{
  parameter_example::OnChange srv;
  srv.request.name = name;
  srv.request.data = data;

  ParameterListeners::iterator srv_name = onChangeListeners[name].begin();
  ParameterListeners::iterator end = onChangeListeners[name].end();
  for (; srv_name != end; ++srv_name) {
    if ( *srv_name != no_call)
      if (!ros::service::call(*srv_name, srv))
        ROS_ERROR_STREAM("Service " << *srv_name << " is not available");
      else
        if (!srv.response.accepted) return false;
  }
  return true;
}

void
ParameterServer::notify_update(const std::string& name, const Data& data, const std::string& no_call)
{
  parameter_example::OnUpdate srv;
  srv.request.name = name;
  srv.request.data = data;

  ParameterListeners::iterator srv_name = onUpdateListeners[name].begin();
  ParameterListeners::iterator end = onUpdateListeners[name].end();
  for (; srv_name != end; ++srv_name) {
    if ( *srv_name != no_call)
      if (!ros::service::call(*srv_name, srv)) {
        ROS_ERROR_STREAM("Service " << *srv_name << " is not available");
      }
  }  
}

bool 
ParameterServer::get_handler(parameter_example::Get::Request  &req,
                             parameter_example::Get::Response &res)
{
  std::pair<DataMap::iterator, bool> ret( dataMap.insert( std::make_pair(req.name, req.data) ) );
  res.data = ret.first->second;
  return true;
}

bool 
ParameterServer::add_on_update_handler(parameter_example::AddOnUpdate::Request  &req,
                                       parameter_example::AddOnUpdate::Response &res)
{
  onUpdateListeners[req.name].insert(req.service);
  return true;
}

bool
ParameterServer::remove_on_update_handler(parameter_example::RemoveOnUpdate::Request  &req,
                                          parameter_example::RemoveOnUpdate::Response &res)
{
  onUpdateListeners[req.name].erase(req.service);
  return true;
}

bool
ParameterServer::add_on_change_handler(parameter_example::AddOnChange::Request  &req,
                                       parameter_example::AddOnChange::Response &res)
{
  onChangeListeners[req.name].insert(req.service);
  return true;
}

bool
ParameterServer::remove_on_change_handler(parameter_example::AddOnChange::Request  &req,
                                          parameter_example::AddOnChange::Response &res)
{
  onChangeListeners[req.name].erase(req.service);
  return true;
}

} // parameter_server
