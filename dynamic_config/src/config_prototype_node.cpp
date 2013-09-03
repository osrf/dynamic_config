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
#include <ros/this_node.h>
#include <ros/callback_queue.h>

#include <boost/variant.hpp>

#include "dynamic_config/Parameter.h"
#include "dynamic_config/Config.h"
#include "dynamic_config/SetConfig.h"
#include "dynamic_config/GetConfig.h"

namespace gsoc {

  namespace configuration {

    template <typename T>
    struct ParameterInfo {
      std::string name;
      std::string parameter;
      T data;
      std::string description;
      std::string owner;

      bool operator==(const ParameterInfo& rhs) {
        return name == rhs.name && 
               parameter == rhs.parameter && 
               data == rhs.data &&
               description == rhs.description &&
               owner == rhs.owner;
      }

      bool operator!=(const ParameterInfo& rhs) {
        return !(*this == rhs);
      }
    };
    typedef boost::variant<ParameterInfo<int>, ParameterInfo<float>, ParameterInfo<double>, 
      ParameterInfo<long>, ParameterInfo<bool>, ParameterInfo<std::string> > ParameterInfoVar;

    typedef std::map<std::string, ParameterInfoVar> ParameterInfoVarMap;

    template <typename T>
    class Parameter {
    public:
      Parameter(const ParameterInfo<T>& info)
      : info_(info) { }

      template <typename U>
      bool operator==(const Parameter<U> rhs) {
        return typeid(T) == typeid(U) && info_ == rhs;
      }

      const std::string& name() const {
        return info_.name;
      }

      const std::string& parameter() const {
        return info_.parameter;
      }

      const T& data() const {
        return info_.data;
      }

      const std::string& description() const {
        return info_.description;
      }

      bool valid() {
        return info_.name != "";
      }

    private:
      ParameterInfo<T> info_;
    };

    class Configuration {
    public:
      template <typename InputIterator>
      Configuration(InputIterator begin, InputIterator end)
      : params_(begin, end) { }

      bool operator==(const Configuration& rhs) {
        return true;
      }

      template <typename T>
      Parameter<T> parameter(const std::string& name) {
        try {
          ParameterInfoVarMap::const_iterator it = params_.find(name);
          const ParameterInfo<T>& info = (it != params_.end() ? boost::get<ParameterInfo<T> >(it->second)
                                                              : emptyParameterInfo<T>());
          return Parameter<T>(info);
        } catch (boost::bad_get& bg) {
          ROS_ERROR_STREAM("Bad cast of parameter " << name);
          return Parameter<T>(emptyParameterInfo<T>());
        }
      }

      template <typename Visitor>
      void introspect(const std::string& name, Visitor visitor) {
        ParameterInfoVarMap::iterator it = params_.find(name);
        if (it != params_.end())
          boost::apply_visitor(boost::bind(CallVisitor(), _1, visitor), it->second);
        else
          ROS_ERROR_STREAM("Parameter " << name << " does not exist");
      }

      template <typename Visitor>
      void introspect(Visitor visitor) {
        ParameterInfoVarMap::iterator it = params_.begin();
        ParameterInfoVarMap::iterator end = params_.end();
        for (; it != end; ++it)
          boost::apply_visitor(boost::bind(CallVisitor(), _1, visitor), it->second);
      }

    private:

      template <typename Visitor>
      void applyVisitorToIterator(ParameterInfoVar& infoVar, Visitor visitor) const {
        boost::apply_visitor(boost::bind(CallVisitor(), _1, visitor), infoVar);
      }

      struct CallVisitor : public boost::static_visitor<> {
        template <typename T, typename Visitor>
        void operator()(const ParameterInfo<T>& info, Visitor visitor) const {
          visitor(Parameter<T>(info));
        }
      };

      template <typename T>
      ParameterInfo<T> emptyParameterInfo() {
        ParameterInfo<T> info;
        return info;
      }

      ParameterInfoVarMap params_;
      std::string md5_;
    };

    class ConfigurationBuilder {
    public:
      ConfigurationBuilder(const ros::NodeHandle& n)
      : n_(n) { }

      template<typename T>
      ConfigurationBuilder& addParameter(const std::string& name,
                                         const std::string& parameter,
                                         const T& default_value,
                                         const std::string& description) {
        ParameterInfo<T> info;
        info.name = name;
        info.description = description;
        info.owner = "";
        n_.param(parameter, info.data, default_value);
        parameterFullName(parameter, info.parameter);

        if (!params_.insert(std::make_pair(name, info)).second)
          ROS_ERROR_STREAM("Parameter " << name << " already exists");
        return *this;
      }

      Configuration build() {
        return Configuration(params_.begin(), params_.end());
      }

    private:

      void parameterFullName(const std::string& parameter, std::string& fullName) {
        if (parameter == "" || parameter[0] == '/')
          fullName = parameter;
        else if (parameter[0] == '~')
          fullName = ros::this_node::getName() + "/" + parameter;
        else
          fullName = n_.getNamespace() + "/" + parameter;
      }

      ros::NodeHandle n_;
      ParameterInfoVarMap params_;
    };

    namespace {

      struct ConfigurationToMsg {

        template <typename T>
        void operator()(Parameter<T> p) {

        }



      };

    } // anonymous

    class ConfigurationServer {
    public:
      typedef boost::function<bool (const Configuration&, const Configuration&, 
                              const std::map<std::string,bool>&)> Callback;

      ConfigurationServer(ros::NodeHandle& n, const Configuration& conf, Callback cb)
      : conf_(conf)
      , cb_(cb)
      , getSrv(n.advertiseService("get_config", &ConfigurationServer::get_config, this))
      , setSrv(n.advertiseService("set_config", &ConfigurationServer::get_config, this))
      , publisher(n.advertise<dynamic_config::Config>("config", 1000, true))
      {
        if (!validServicesAndPublisher())
          shutdown();
      }

      ~ConfigurationServer() { }

      bool reconfigure(const Configuration& newConf) {
        return true;
      }

      const Configuration& configuration() const {
        return conf_;
      }

    private:

      bool get_config(dynamic_config::GetConfig::Request  &req,
                      dynamic_config::GetConfig::Response &res) {
        return true;
      }

      bool set_config(dynamic_config::SetConfig::Request  &req,
                      dynamic_config::SetConfig::Response &res) {
        // call callback
        //std::map<std::string,bool> myMap;
        //res.accepted = cb_(conf_, conf_, myMap);
        //if (res.accepted)
        //  publisher.publish(req.config);
        return true;
      }

      bool validServicesAndPublisher() {
        return getSrv && setSrv && publisher;
      }

      void shutdown() {
        getSrv.shutdown();
        setSrv.shutdown();
        publisher.shutdown();
      }

      Configuration conf_;
      Callback cb_;
      ros::ServiceServer getSrv;
      ros::ServiceServer setSrv;
      ros::Publisher publisher;
    };

    class NodeHandle {
    public:
      NodeHandle(const ros::NodeHandle& n)
      : n_(n)
      { }

      ConfigurationBuilder createConfiguration() const {
        return ConfigurationBuilder(n_);
      }

      ConfigurationServer createConfigurationServer(const Configuration& conf,
                                                    ConfigurationServer::Callback cb) {
        return ConfigurationServer(n_, conf, cb);
      }

    private:
      ros::NodeHandle n_;
    };

  } // configuration

} // gsoc

// All accepted types need a operator() function. It be generalized
// with templates.
struct Introspection {

  void operator()(gsoc::configuration::Parameter<std::string> p) const {
    ROS_INFO_STREAM("Param " << p.name() << " is a string with value " << p.data());
  }

  void operator()(gsoc::configuration::Parameter<int> p) const {
    ROS_INFO_STREAM("Param " << p.name() << " is an int with value " << p.data());
  }

  template <typename T>
  void operator()(gsoc::configuration::Parameter<T> p) const {
    ROS_INFO_STREAM("Param " << p.name() << " using templates");
  }

};

bool acceptChanges(const gsoc::configuration::Configuration& current, 
                   const gsoc::configuration::Configuration& candidate,
                   const std::map<std::string, bool>& changes) {
  return true;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "config_prototype_node");

  namespace config = gsoc::configuration;

  // Server executes in its own thread to simulate
  // it is a different node
  ros::NodeHandle srvRosHandle("server");
  ros::CallbackQueue srvQueue;
  srvRosHandle.setCallbackQueue(&srvQueue);
  ros::AsyncSpinner srvSpinner(1, &srvQueue);
  srvSpinner.start();

  // This is my own NodeHandle. In a future it can be
  // integrated in the real NodeHandle
  config::NodeHandle srvHandle(srvRosHandle);

  // Create the configuration of the node "/server"
  config::Configuration configuration = srvHandle.createConfiguration()
    .addParameter("param1", "/server/param1", std::string("My default value"), "This is the description")
    .addParameter("param2", "/server/param2", 100, "This is an int")
    .build();

  config::Parameter<std::string> param1 = configuration.parameter<std::string>("param1");
  ROS_ASSERT(param1.valid());
  ROS_ASSERT("param1" == param1.name());
  ROS_ASSERT("/server/param1" == param1.parameter());
  ROS_ASSERT("My default value" == param1.data());
  ROS_ASSERT("This is the description" == param1.description());

  // If user tries to cast a parameter to a wrong type. A message is shown and an empty
  // parameter is returned.
  config::Parameter<float> wrongTypeParam = configuration.parameter<float>("param1");
  ROS_ASSERT(!wrongTypeParam.valid());

  // Introspection of the configuration. Can be for just one parameter
  // or the whole configuration
  ROS_INFO("Introspection of the configuration");
  configuration.introspect("param1", Introspection());
  configuration.introspect(Introspection());

  // Publish the configuration
  config::ConfigurationServer srv = srvHandle.createConfigurationServer(configuration, acceptChanges);


  ROS_INFO("Test finished!!");
  return 0;
}