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

#ifndef DYNAMIC_CONFIG_CONFIGURATION_BUILDER_H
#define DYNAMIC_CONFIG_CONFIGURATION_BUILDER_H

#include <ros/ros.h>
#include <boost/regex.hpp>

#include "gsoc/configuration/configuration.h"
 
namespace gsoc {

  namespace configuration {

    struct Line : public std::string { };

    std::istream &operator>>(std::istream &is, Line &l);

    class ConfigurationBuilder {
    public:

      ConfigurationBuilder(const ros::NodeHandle& n);

      template <typename T>
      ConfigurationBuilder& addParameter(const std::string& name,
                                         const std::string& global_param,
                                         const T& default_value) {
        T value;
        n_.param(global_param, value, default_value);
        conf_.put(name, value);
        return *this;
      }

      template <typename T>
      ConfigurationBuilder& addParameter(const std::string& name,
                                         const T& default_value) {
        conf_.put(name, default_value);
        return *this;
      }

      ConfigurationBuilder& addParameters(std::istream_iterator<Line> is);

      const Configuration& build() const;

    private:

    struct FromString {
      FromString(Configuration& conf);

      void put(const std::string& line);

      void put(const std::string& name, const std::string& value);
      
      Configuration& conf;
    };

      ros::NodeHandle n_;
      Configuration conf_;
    };

    ConfigurationBuilder make_builder(const ros::NodeHandle& n);

    ConfigurationBuilder make_builder(const std::string& namespc = "~");
 
  } // configuration

} // gsoc

#endif
