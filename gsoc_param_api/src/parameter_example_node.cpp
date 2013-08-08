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
#include <boost/signals2.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "gsoc_param_api/AcceptParameter.h"
#include "gsoc_param_api/UpdateParameter.h"

template <typename T>
class Parameter
{
public:
  Parameter(const std::string& name, const T& data) 
  : name_(name)
  , data_(data)
  { }

  const T& get() const
  { return data_; }

  const std::string& name() const
  { return name_; }

private:
  std::string name_;
  T data_;
};

class ParamNodeHandle
{
public:
  ParamNodeHandle()
  : impl_(new Impl)
  { }

  ParamNodeHandle(const std::string& namespc)
  : impl_(new Impl(namespc))
  { }

  void createParameter() {}
  void createAcceptant() {}
  void createUpdater() {}

private:
  struct Impl {
    Impl() 
    : handle_("~")
    , spinner_(0, &queue_) 
    { init(); }

    Impl(const std::string& namespc) 
    : handle_(namespc)
    , spinner_(0, &queue_) 
    { init(); }

    void init() {
      handle_.setCallbackQueue(&queue_);
      spinner_.start();
      ROS_INFO_STREAM("Init " << handle_.getNamespace() << " parameters");
    }

    ros::NodeHandle handle_;
    ros::CallbackQueue queue_;
    ros::AsyncSpinner spinner_;
  };

  bool accept_parameter_callback(gsoc_param_api::AcceptParameter::Request  &req,
                                 gsoc_param_api::AcceptParameter::Response &res)
  {
    return true;
  }

  bool node_a_update_parameter_callback(gsoc_param_api::UpdateParameter::Request  &req,
                                        gsoc_param_api::UpdateParameter::Response &res)
  {
    return true;
  }

  typedef boost::shared_ptr<Impl> ImplPtr;
  ImplPtr impl_;
};


class ParameterGroup
{
public:
  ParameterGroup(const std::string& namespc)
  : namespc_(namespc)
  , handle_(namespc)
  { }

  template <typename T>
  Parameter<T> getParameter(const std::string& name, const T& default_data) {
    return Parameter<T>(name, default_data);
  }

private:
  std::string namespc_;
  ros::NodeHandle handle_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "parameter_example_node");

  ParamNodeHandle node_a_handle("/node_a");
  ParamNodeHandle node_b_handle("/node_b");

  // Create a local copy of a parameter with a default value
  {
    ParameterGroup group("/node_a");
    Parameter<std::string> param = group.getParameter("param1", std::string("my default value"));
    ROS_ASSERT("my default value" == param.get());
  }

  // Node A creates a Parameter and nodes B gets a copy of the Parameter
  {
    
  }

  ROS_INFO("Test finished correctly");
}