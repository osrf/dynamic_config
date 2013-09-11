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

#include <map>
#include <iterator>
#include <algorithm>
#include <boost/variant.hpp>

namespace gsoc {

  namespace configuration {

    namespace {

      typedef boost::variant<std::string, int> Parameter;
      typedef std::map<std::string, Parameter> Parameters;

      template <typename Result, typename Visitor>
      struct  UnaryOperation : boost::static_visitor<Result> {

        UnaryOperation(const std::string& name, const Visitor& visitor)
        : name_(name)
        , visitor_(visitor)
        { }

        template <typename T> 
        Result operator()(const T& t) const {
          return visitor_(std::make_pair(name_, t));
        }

        const std::string& name_;
        const Visitor& visitor_;
      };

      template <typename Result, typename Visitor>
      struct Operation {

        Operation(const Visitor& visitor)
        :visitor_(visitor)
        { }

        Result operator()(const std::pair<const std::string, Parameter>& pair) const {
          UnaryOperation<Result,Visitor> op(pair.first, visitor_);
          return boost::apply_visitor(op, pair.second);
        }

        const Visitor& visitor_;
      };

      template <typename Result, typename Visitor>
      Operation<Result,Visitor> make_operation(const Visitor& visitor) {
        return Operation<Result,Visitor>(visitor);
      }

      bool sameName(std::pair<const std::string, Parameter> p1,
                    std::pair<const std::string, Parameter> p2) {
        return p1.first == p2.first;
      }

      bool sameType(std::pair<const std::string, Parameter> p1,
                    std::pair<const std::string, Parameter> p2) {
        return p1.second.type() == p2.second.type();
      }

      bool sameValue(std::pair<const std::string, Parameter> p1,
                    std::pair<const std::string, Parameter> p2) {
        return p1.second == p2.second;
      }

    } // anonymous

    class Configuration {
    public:
      Configuration() { }

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

      bool has(const std::string& name) const {
        Parameters::const_iterator it = params_.find(name);
        return notEnd(it);
      }

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
          std::cerr << "Param " << name << " does not exist" << std::endl;
      }

      template <typename ResultType, typename Visitor>
      ResultType apply(const std::string& name, Visitor visitor) const {
        std::vector<ResultType> result(1);
        Parameters::const_iterator it = params_.find(name);
        if (it != params_.end())
          std::transform(it, ++it, result.begin(), make_operation<ResultType>(visitor));
        else
          std::cerr << "Param " << name << " does not exist" << std::endl;
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

      int size() const {
        return params_.size();
      }

      bool equivalent(const Configuration& conf) const {
        return params_.size() == conf.params_.size() &&
               std::equal(params_.begin(), params_.end(), conf.params_.begin(), sameName) &&
               std::equal(params_.begin(), params_.end(), conf.params_.begin(), sameType);
      }

      bool operator==(const Configuration& rhs) const {
        return equivalent(rhs) &&
               std::equal(params_.begin(), params_.end(), rhs.params_.begin(), sameValue);
      }

      bool operator!=(const Configuration& rhs) const {
        return !(*this == rhs);
      }

    private:
      bool notEnd(Parameters::const_iterator& it) const {
        return it != params_.end();
      }

      template <typename T>
      bool typeCorrect(Parameters::const_iterator& it) const {
        return (it->second).type() == typeid(T);
      }

      Parameters params_;
    };

    class ConfigurationBuilder {
    public:
      ConfigurationBuilder(const ros::NodeHandle& n)
      : n_(n)
      { }

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

      const Configuration& build() const {
        return conf_;
      }

    private:
      ros::NodeHandle n_;
      Configuration conf_;
    };

    ConfigurationBuilder make_builder(const ros::NodeHandle& n) {
      return ConfigurationBuilder(n);
    }

    ConfigurationBuilder make_builder(const std::string& namespc = "~") {
      return make_builder(ros::NodeHandle(namespc));
    }

  } // configuration

} // gsoc

#endif
