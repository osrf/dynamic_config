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

#ifndef DYNAMIC_CONFIG_MSG_HANDLER_H
#define DYNAMIC_CONFIG_MSG_HANDLER_H

#include <ros/ros.h>
#include "gsoc/configuration/serialization.h"
#include <dynamic_config/Param.h>
#include <dynamic_config/Conf.h>


namespace gsoc {

  namespace configuration {

    namespace msg_handler {

      struct TypeToInt {
        template <typename T>
        int operator()(const T& t) 
        { ROS_ERROR_STREAM("TypeToInt unknown type");
          return -1; }
        int operator()(const std::string& t) 
        { return 0; }
        int operator()(const int& t)
        { return 1; }
        int operator()(const double& t)
        { return 2; }
      };

      // Very ugly.
      template <class Configuration>
      void deserializeInConf(const dynamic_config::Param& param, Configuration& conf) {
        switch (param.type) {
          case 0: conf.put(param.name, serialization::deserialize<std::string>(param.data));
            break;
          case 1: conf.put(param.name, serialization::deserialize<int>(param.data));
            break;
          case 2: conf.put(param.name, serialization::deserialize<double>(param.data));
            break;
          default: ROS_ERROR_STREAM("Deserializing unknwon type");
        };
      }

      struct ParameterToParamMsg {
        template <typename T>
        dynamic_config::Param operator()(const std::pair<std::string,T> pair) const {
          dynamic_config::Param param;
          param.name = pair.first;
          serialization::serialize(pair.second, param.data);
          TypeToInt t;
          param.type = t(pair.second);
          return param;
        }
      };

      template <typename InputIterator, typename Configuration>
      void paramMsgToParameter(InputIterator first, InputIterator last, Configuration& conf) {
        while (first != last) {
          deserializeInConf(*first++, conf);
        }
      }

    } // msg_handler

  } // configuration

} // gsoc

#endif
