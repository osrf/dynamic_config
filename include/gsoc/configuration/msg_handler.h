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

      enum ParamTypes { BOOL, STRING, INT, LONG, FLOAT, DOUBLE, UNKNOWN };

      struct TypeToInt {
        int operator()(const bool&)
        { return BOOL; }
        int operator()(const std::string&) 
        { return STRING; }
        int operator()(const int&)
        { return INT; }
        int operator()(const long&)
        { return LONG; }
        int operator()(const float&)
        { return FLOAT; }
        int operator()(const double&)
        { return DOUBLE; }
        template <typename T>
        int operator()(const T& t) 
        { ROS_ERROR_STREAM("TypeToInt unknown type");
          return UNKNOWN; }
      };

      // Very ugly.
      template <class Configuration>
      void deserializeInConf(const dynamic_config::Param& param, Configuration& conf) {
        switch (param.type) {
          case BOOL: conf.put(param.name, serialization::deserialize<bool>(param.data));
            break;
          case STRING: conf.put(param.name, serialization::deserialize<std::string>(param.data));
            break;
          case INT: conf.put(param.name, serialization::deserialize<int>(param.data));
            break;
          case LONG: conf.put(param.name, serialization::deserialize<long>(param.data));
            break;
          case FLOAT: conf.put(param.name, serialization::deserialize<float>(param.data));
            break;
          case DOUBLE: conf.put(param.name, serialization::deserialize<double>(param.data));
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

      template <typename Container, typename Configuration>
      void paramMsgToParameter(Container container, Configuration& conf) {
        paramMsgToParameter(container.begin(), container.end(), conf);
      }

    } // msg_handler

  } // configuration

} // gsoc

#endif
