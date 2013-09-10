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

#include <ros/ros.h>

#include <dynamic_config/GetConf.h>
#include <dynamic_config/SetConf.h> 


namespace gsoc {

  namespace configuration {

    bool accept_all(gsoc::configuration::Configuration&) {
      return true;
    }

    bool deny_all(gsoc::configuration::Configuration&) {
      return false;
    }

    class ConfigurationServer {
    public:
      typedef boost::function<bool (Configuration&)> Callback;

      ConfigurationServer(ros::NodeHandle& n, Configuration& conf, Callback cb = accept_all) 
      : conf_(conf)
      , cb_(cb)
      , getSrv_(n.advertiseService("get_conf", &ConfigurationServer::getSrvCallback, this))
      , setSrv_(n.advertiseService("set_conf", &ConfigurationServer::setSrvCallback, this))
      , publisher_(n.advertise<dynamic_config::Conf>("conf", 100, true))
      { 
        reconfigure(conf);
      }

      Configuration configuration() {
        return conf_;
      }

      bool reconfigure(Configuration& conf) {
        if (!conf_.equivalent(conf)) {
          ROS_ERROR("Configuration structure not valid");
          return false;
        }
        bool accepted = cb_(conf);
        if (accepted) {
          conf_ = conf;
          dynamic_config::Conf msg;
          msg_handler::ParameterToParamMsg visitor;
          conf.applyAll<dynamic_config::Param>(visitor, std::inserter(msg.params, msg.params.begin()));
          publisher_.publish(msg);
        }
        return accepted;
      }

    private:

      bool getSrvCallback(dynamic_config::GetConf::Request&  req,
                          dynamic_config::GetConf::Response& res) {
        conf_.applyAll<dynamic_config::Param>(msg_handler::ParameterToParamMsg(),
          std::inserter(res.conf.params, res.conf.params.begin()));
        return true;
      }

      bool setSrvCallback(dynamic_config::SetConf::Request&  req,
                          dynamic_config::SetConf::Response& res) {
        Configuration conf;
        msg_handler::paramMsgToParameter(req.conf.params.begin(),
                                         req.conf.params.end(), conf);
        res.accepted  = reconfigure(conf);
        return true;
      }

      Configuration conf_;
      Callback cb_;
      ros::ServiceServer getSrv_;
      ros::ServiceServer setSrv_;
      ros::Publisher publisher_;
    };

  } // configuration

} // gsoc