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

#include "gsoc/configuration/configuration_client.h"

namespace gsoc {

  namespace configuration {

      ConfigurationClient::ConfigurationClient(ros::NodeHandle n)
      : impl_(new Impl)
      {
        impl_->getConfClient_ = n.serviceClient<dynamic_config::GetConf>("get_conf");
        impl_->setConfClient_ = n.serviceClient<dynamic_config::SetConf>("set_conf");

        if (!impl_->valid()) {
          shutdown();
          ROS_ERROR_STREAM("Cannot create configuration client of node " << n.getNamespace());
        }
      }

      ConfigurationClient::ConfigurationClient(const ConfigurationClient& other)
      : impl_(other.impl_)
      { }

      ConfigurationClient& ConfigurationClient::operator=(const ConfigurationClient& rhs)
      {
        impl_ = rhs.impl_;
        return *this;
      }

      bool ConfigurationClient::operator==(const ConfigurationClient& rhs) const
      {
        return impl_ == rhs.impl_;
      }

      bool ConfigurationClient::operator!=(const ConfigurationClient& rhs) const
      {
        return impl_ != rhs.impl_;
      }

      bool ConfigurationClient::operator<(const ConfigurationClient& rhs) const
      {
        return impl_ < rhs.impl_;
      }

      ConfigurationClient::operator void*() const {
        return impl_->valid() ? (void*)1 : (void*)0;
      }

      void ConfigurationClient::shutdown()
      {
        impl_->getConfClient_.shutdown();
        impl_->setConfClient_.shutdown();
      }

      Configuration ConfigurationClient::configuration() {
        Configuration conf;
        dynamic_config::GetConf srv;
        if (impl_->getConfClient_.call(srv))
          msg_handler::paramMsgToParameter(srv.response.conf.params, conf);
        else
          ROS_ERROR_STREAM("Cannot get configuration from " << impl_->getConfClient_.getService());
        return conf;
      }

      bool ConfigurationClient::reconfigure(const Configuration& conf) {
        dynamic_config::SetConf srv;
        conf.applyAll<dynamic_config::Param>(msg_handler::ParameterToParamMsg(), 
                                             make_inserter_at_beginning(srv.request.conf.params));

        if (impl_->setConfClient_.call(srv))
          return srv.response.accepted;

        ROS_ERROR_STREAM("Cannot reconfigure " << impl_->setConfClient_.getService());
        return false;
      }

      bool ConfigurationClient::Impl::valid() const
      {
        return getConfClient_ && setConfClient_;
      }

  } // configuration

} // gsoc
