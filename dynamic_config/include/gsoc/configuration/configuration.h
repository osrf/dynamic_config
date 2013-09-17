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

#ifndef DYNAMIC_CONFIG_CONFIGURATION_H
#define DYNAMIC_CONFIG_CONFIGURATION_H

#include <ros/console.h>

#include <map>
#include <algorithm>
#include <boost/variant.hpp>

#include "gsoc/configuration/configuration_helper.h"
#include "gsoc/configuration/persistance.h"

namespace gsoc {

  namespace configuration {

    class Configuration {

      typedef boost::variant<bool, std::string, int, long, float, double> Parameter;
      typedef std::map<std::string, Parameter> Parameters;

    public:

      template <typename T>
      T get(const std::string& name) const {
        Parameters::const_iterator it = params_.find(name);
        if (notEnd(it) && typeCorrect<T>(it)) {
          return boost::get<T>(it->second);
        }
        ROS_ERROR_STREAM("Parameter " << name << " does not exist or type is wrong");
        return T();
      }

      template <typename T>
      void put(const std::string& name, const T& t) {
        params_[name] = t;
      }

      template <typename T>
      bool insert(const std::string& name, const std::string& value) {
        bool result = persistance::isType<T>(value);
        if (result) put(name, persistance::cast<T>(value));
        return result;
      }

      void insert(const std::string& name, const std::string& value);

      void insert(const std::string& line);

      bool has(const std::string& name) const;

      template <typename T>
      bool isType(const std::string& name) const {
        Parameters::const_iterator it = params_.find(name);
        return typeCorrect<T>(it);
      }

      template <typename Visitor>
      void apply(const std::string& name, Visitor visitor) const {
        Parameters::const_iterator it = params_.find(name);
        if (it != params_.end())
          std::for_each(it, ++it, make_operation<void>(visitor));
        else
          ROS_ERROR_STREAM("Param " << name << " does not exist");
      }

      template <typename ResultType, typename Visitor>
      ResultType apply(const std::string& name, Visitor visitor) const {
        std::vector<ResultType> result(1);
        Parameters::const_iterator it = params_.find(name);
        if (it != params_.end())
          std::transform(it, ++it, result.begin(), make_operation<ResultType>(visitor));
        else
          ROS_ERROR_STREAM("Param " << name << " does not exist");
        return result[0];
      }

      template <typename Visitor>
      void applyAll(Visitor visitor) const {
        std::for_each(params_.begin(), params_.end(), make_operation<void>(visitor));
      }

      template <typename ResultType, typename Visitor, typename OutputIterator>
      void applyAll(Visitor visitor, OutputIterator result) const {
        std::transform(params_.begin(), params_.end(), result, make_operation<ResultType>(visitor));
      }

      int size() const;

      template <typename OutputIterator>
      void names(OutputIterator result) const {
        std::transform(params_.begin(), params_.end(), result, parameterName);
      }

      bool equivalent(const Configuration& conf) const;

      bool operator==(const Configuration& rhs) const;

      bool operator!=(const Configuration& rhs) const;

    private:

      bool notEnd(Parameters::const_iterator& it) const;
      
      template <typename T>
      bool typeCorrect(Parameters::const_iterator& it) const {
        return (it->second).type() == typeid(T);
      }

      Parameters params_;
    };

  } // configuration

} // gsoc

#endif
