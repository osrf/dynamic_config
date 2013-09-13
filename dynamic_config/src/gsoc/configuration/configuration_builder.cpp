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

#include "gsoc/configuration/configuration_builder.h"

namespace gsoc {

  namespace configuration {

    std::istream &operator>>(std::istream &is, Line &l)
    {
      std::getline(is, l);
      return is;
    }

    ConfigurationBuilder::ConfigurationBuilder(const ros::NodeHandle& n)
    : n_(n)
    { }

    ConfigurationBuilder& ConfigurationBuilder::addParameters(std::istream_iterator<Line> is)
    {
      std::istream_iterator<Line> end;
      FromString fromString(conf_);
      for (; is != end; ++is)
        if (!is->empty()) fromString.put(*is);
      return *this;
    }

    const Configuration& ConfigurationBuilder::build() const
    {
      return conf_;
    }

    ConfigurationBuilder::FromString::FromString(Configuration& conf)
    :conf(conf)
    { }

    void ConfigurationBuilder::FromString::put(const std::string& line)
    {
      int pos = line.find("=");
      if (pos != std::string::npos)
        put(line.substr(0, pos),
            line.substr(pos+1, line.length())); 
    }

    void ConfigurationBuilder::FromString::put(const std::string& name, 
                                               const std::string& value)
    {
      if (name.empty() || value.empty()) {
        ROS_ERROR_STREAM("Parameters can't be empty");
        return;
      }

      // Boolean
      if (boost::regex_match(value, boost::regex("true|false")))
        conf.put(name, value == "true");
      // String
      else if (boost::regex_match(value, boost::regex("\".*\"")))
        conf.put(name, std::string(++value.begin(), --value.end()));          
      // Int
      else if (boost::regex_match(value, boost::regex("-?[0-9]+")))
        conf.put(name, boost::lexical_cast<int>(value));
      // Long
      else if (boost::regex_match(value, boost::regex("-?[0-9]+l")))
        conf.put(name, boost::lexical_cast<long>(std::string(value.begin(), --value.end())));
      // Float
      else if (boost::regex_match(value, boost::regex("-?[0-9]*\\.[0-9]+f")) ||
               boost::regex_match(value, boost::regex("-?[0-9]+\\.[0-9]*f")) )
        conf.put(name, boost::lexical_cast<float>(std::string(value.begin(), --value.end())));
      // Double
      else if (boost::regex_match(value, boost::regex("-?[0-9]*\\.[0-9]+")) ||
               boost::regex_match(value, boost::regex("-?[0-9]+\\.[0-9]*")) )
        conf.put(name, boost::lexical_cast<double>(value));
      // Not supported type
      else
        ROS_ERROR_STREAM("No match for " << name << "=" << value);
    }

    ConfigurationBuilder make_builder(const ros::NodeHandle& n)
    {
      return ConfigurationBuilder(n);
    }

    ConfigurationBuilder make_builder(const std::string& namespc)
    {
      return make_builder(ros::NodeHandle(namespc));
    }

  } // configuration

} // gsoc
