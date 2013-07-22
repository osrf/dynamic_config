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

#include "ros_parameter/parameter_server_listener.h"

namespace ros_parameter {

ParameterServerListener::ParameterServerListener()
{
  ros::NodeHandle n("~");
  serviceServers_.push_back( n.advertiseService("on_update", &ParameterServerListener::on_update_handler, this) );
  on_update_service_ = serviceServers_.back().getService();
  serviceServers_.push_back( n.advertiseService("on_change", &ParameterServerListener::on_change_handler, this) );
  on_change_service_ = serviceServers_.back().getService();
}

void
ParameterServerListener::on_update(OnUpdateCallback& callback)
{ onUpdateCallback_ = callback; }

void
ParameterServerListener::on_update( void(*func)(const std::string&, const Data&) )
{ 
  OnUpdateCallback cb = func;
  on_update(cb); 
}

void
ParameterServerListener::on_change(OnChangeCallback& callback)
{ onChangeCallback_ = callback; }

void
ParameterServerListener::on_change( bool(*func)(const std::string&, const Data&) )
{ 
  OnChangeCallback cb = func;
  on_change(cb);
}

const std::string&
ParameterServerListener::on_update_service() const
{ return on_update_service_; }

const std::string&
ParameterServerListener::on_change_service() const
{ return on_change_service_; }

bool
ParameterServerListener::on_update_handler(parameter_example::OnUpdate::Request  &req,
                                           parameter_example::OnUpdate::Response &res)
{
  if (onUpdateCallback_)
    onUpdateCallback_(req.name, req.data);
  return true;
}

bool
ParameterServerListener::on_change_handler(parameter_example::OnChange::Request  &req,
                                           parameter_example::OnChange::Response &res)
{
  res.accepted = true;
  if (onChangeCallback_)
    res.accepted = onChangeCallback_(req.name, req.data);
  return true;
}

} // ros_parameter
