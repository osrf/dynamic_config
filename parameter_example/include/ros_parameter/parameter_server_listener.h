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

#include <boost/function.hpp>
#include "ros/ros.h"

#include "parameter_example/OnUpdate.h"
#include "parameter_example/OnChange.h"

namespace ros_parameter {

class ParameterServerListener
{
  typedef std::vector<uint8_t> Data;

public:
  typedef boost::function<void (const std::string&, const Data&)> OnUpdateCallback;
  typedef boost::function<bool (const std::string&, const Data&)> OnChangeCallback;

  ParameterServerListener();

  void on_update(OnUpdateCallback& callback);
  void on_update( void(*func)(const std::string&, const Data&) );

  void on_change(OnChangeCallback& callback);
  void on_change( bool(*func)(const std::string&, const Data&) );

  const std::string& on_update_service() const;
  const std::string& on_change_service() const;

private:

  bool on_update_handler(parameter_example::OnUpdate::Request  &req,
                         parameter_example::OnUpdate::Response &res);

  bool on_change_handler(parameter_example::OnChange::Request  &req,
                         parameter_example::OnChange::Response &res);

  std::vector<ros::ServiceServer> serviceServers_;
  std::string on_update_service_;
  std::string on_change_service_;
  OnUpdateCallback onUpdateCallback_;
  OnChangeCallback onChangeCallback_;
};

} // ros_parameter
