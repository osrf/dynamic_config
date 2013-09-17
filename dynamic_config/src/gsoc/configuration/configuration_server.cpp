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

#include "gsoc/configuration/configuration_server.h"

namespace gsoc {

  namespace configuration {

    bool accept_all(const Configuration&) {
      return true;
    }

    bool deny_all(const Configuration&) {
      return false;
    }

    ConfigurationServer::ConfigurationServer(ros::NodeHandle& n, Configuration& conf, Callback cb) 
    : impl_(new Impl)
    {
      impl_->conf_ = conf;
      impl_->cb_ = cb;
      impl_->getSrv_ = n.advertiseService("get_conf", &ConfigurationServer::getSrvCallback, this);
      impl_->setSrv_ = n.advertiseService("set_conf", &ConfigurationServer::setSrvCallback, this);
      impl_->publisher_ = n.advertise<dynamic_config::Conf>("conf", 100, true);

      if (impl_->getSrv_ && impl_->setSrv_ && impl_->publisher_) {
        reconfigure(conf);
      } else {
        shutdown();
        ROS_ERROR("Can't create the configuration server");
      }
    }

    ConfigurationServer::ConfigurationServer(const ConfigurationServer& other)
    : impl_(other.impl_)
    { }

    ConfigurationServer& ConfigurationServer::operator=(const ConfigurationServer& rhs)
    {
      impl_ = rhs.impl_;
      return *this;
    }

    bool ConfigurationServer::operator==(const ConfigurationServer& rhs) const
    {
      return impl_ == rhs.impl_;
    }

    bool ConfigurationServer::operator!=(const ConfigurationServer& rhs) const
    {
      return impl_ != rhs.impl_;
    }

    bool ConfigurationServer::operator<(const ConfigurationServer& rhs) const
    {
      return impl_ < rhs.impl_;
    }

    ConfigurationServer::operator void*() const {
      return impl_->getSrv_ && impl_->setSrv_ && impl_->publisher_ ? (void*)1 : (void*)0;
    }

    const Configuration& ConfigurationServer::configuration() const {
      return impl_->conf_;
    }

    bool ConfigurationServer::reconfigure(const Configuration& conf)
    {
      if (!impl_)
        return false;

      if (!impl_->conf_.equivalent(conf)) {
        ROS_ERROR("Configuration structure not valid");
        return false;
      }
      bool accepted = impl_->cb_(conf);
      if (accepted) {
        impl_->conf_ = conf;
        dynamic_config::Conf msg;
        msg_handler::ParameterToParamMsg visitor;
        conf.applyAll<dynamic_config::Param>(visitor, std::inserter(msg.params, msg.params.begin()));
        impl_->publisher_.publish(msg);
      }
      return accepted;
    }

    void ConfigurationServer::shutdown()
    {
      impl_->getSrv_.shutdown();
      impl_->setSrv_.shutdown();
      impl_->publisher_.shutdown();
    }

    bool ConfigurationServer::getSrvCallback(dynamic_config::GetConf::Request&  req,
                                             dynamic_config::GetConf::Response& res) {
      impl_->conf_.applyAll<dynamic_config::Param>(msg_handler::ParameterToParamMsg(),
        std::inserter(res.conf.params, res.conf.params.begin()));
      return true;
    }

    bool ConfigurationServer::setSrvCallback(dynamic_config::SetConf::Request&  req,
                                             dynamic_config::SetConf::Response& res) {
      Configuration conf;
      msg_handler::paramMsgToParameter(req.conf.params.begin(),
                                       req.conf.params.end(), conf);
      res.accepted  = reconfigure(conf);
      return true;
    }

  } // configuration

} // gsoc
