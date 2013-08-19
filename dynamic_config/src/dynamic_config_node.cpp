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
#include <ros/callback_queue.h>

#include <boost/variant.hpp>
#include <boost/fusion/include/map.hpp>
#include <boost/fusion/include/at_key.hpp>

#include "dynamic_config/Parameter.h"
#include "dynamic_config/Config.h"
#include "dynamic_config/SetConfig.h"
#include "dynamic_config/GetConfig.h"

namespace serialization {

  template <typename T>
  void serialize(const T& data, std::vector<uint8_t>& buffer)
  {
    buffer.resize(ros::serialization::serializationLength(data));
    ros::serialization::OStream ostream(&buffer[0], buffer.size());
    ros::serialization::serialize(ostream, data);
  }

  void serialize(const std::vector<uint8_t>& data, std::vector<uint8_t>& buffer)
  { buffer = data; }

  template <typename T>
  std::vector<uint8_t> serialize(const T& data)
  {
    std::vector<uint8_t> buffer;
    serialize(data, buffer);
    return buffer;
  }

  template < typename T >
  void deserialize(std::vector<uint8_t>& data, T& output)
  {
    ros::serialization::IStream istream(&data[0], data.size());
    ros::serialization::Serializer<T>::read(istream, output);
  }

  template <>
  void deserialize<std::vector<uint8_t> >(std::vector<uint8_t>& data, std::vector<uint8_t>& output)
  { output = data; }

  template <typename T>
  T deserialize(std::vector<uint8_t> data)
  {
    T output;
    deserialize(data, output);
    return output;
  }

} // serialization

typedef boost::fusion::map<boost::fusion::pair<int32_t, std::string>,
                           boost::fusion::pair<float, std::string>,
                           boost::fusion::pair<double, std::string>,
                           boost::fusion::pair<std::string, std::string> 
        >  TypeIdMap;
TypeIdMap default_typeIdMap(boost::fusion::make_pair<int32_t>("int32"),
                            boost::fusion::make_pair<float>("float32"),
                            boost::fusion::make_pair<double>("float64"),
                            boost::fusion::make_pair<std::string>("string"));

template <typename T>
std::string getTypeId(const TypeIdMap& map = default_typeIdMap) {
  return boost::fusion::at_key<T>(map);
}

typedef boost::variant<int32_t, float, double, std::string> Data;
struct DataStruct { Data data; std::string description; };
typedef std::map<std::string, DataStruct> ParametersMap;

class Configuration {
public:
  Configuration(const dynamic_config::Config& config)
  : config_(config)
  { }

  ~Configuration() { }

  template <typename T>
  T get(const std::string& name)
  { 
    std::vector<dynamic_config::Parameter>::iterator it = config_.parameters.begin();
    std::vector<dynamic_config::Parameter>::iterator end = config_.parameters.end();
    for (; it != end; ++it) {
      if ( it->name == name ) {
        return serialization::deserialize<T>(it->data);
      }
    }
    T data;
    return data;
  }

  template <typename T>
  void set(const std::string& name, const T& data)
  { 
  }

  std::string getDescription(const std::string& name)
  { 
    return "";
  }

  std::set<std::string> names()
  {
    std::set<std::string> set;
    return set;
  }

  dynamic_config::Config& getConfig()
  {
    return config_;
  }

private:
  dynamic_config::Config config_;
};

class ConfigBuilder {
public:
  static ConfigBuilder make()
  {
    return ConfigBuilder(); 
  }

  template <typename T>
  ConfigBuilder& addParameter(const std::string& name, const T& data, const std::string& description = "")
  {
    return addParameter(name, serialization::serialize(data), getTypeId<T>(), description);
  }

  ConfigBuilder& addParameter(const std::string& name, const std::vector<uint8_t> data,
                              const std::string& type, const std::string& description)
  {
    dynamic_config::Parameter parameter;
    parameter.name = name;
    parameter.data = data;
    parameter.type = type;
    parameter.description = description;
    config_.parameters.push_back(parameter);
    return *this;
  }

  Configuration build()
  {
    return Configuration(config_); 
  }

private:
  dynamic_config::Config config_;
};

class ConfigurationServer {
public:
  typedef boost::function<bool (Configuration&, std::string&)> Callback;

  ConfigurationServer(const ros::NodeHandle& n, Callback cb)
  : n_(n), callback_(cb)
  {
    getServer_ = n_.advertiseService("get_server", &ConfigurationServer::getServerCallback, this);
    setServer_ = n_.advertiseService("set_server", &ConfigurationServer::setServerCallback, this);
    publisher_ = n_.advertise<dynamic_config::Config>("config", 10, true);
  }

  void publish(dynamic_config::Config& config) {
    config_ = config;
    publisher_.publish(config);
  }

private:

  bool getServerCallback(dynamic_config::GetConfig::Request  &req,
                         dynamic_config::GetConfig::Response &res)
  {
    res.config = config_;
    return true;
  }

  bool setServerCallback(dynamic_config::SetConfig::Request  &req,
                         dynamic_config::SetConfig::Response &res)
  {
    Configuration conf(req.config);
    res.accepted = callback_(conf, res.reason);
    if (res.accepted) {
      publish(req.config);
    }
    return true;
  }

  ros::NodeHandle n_;
  Callback callback_;
  dynamic_config::Config config_;
  ros::ServiceServer getServer_;
  ros::ServiceServer setServer_;
  ros::Publisher publisher_;
};

bool set_configuration_callback(Configuration& config, std::string& reason) {
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_config_node");

  Configuration config = ConfigBuilder::make()
   .addParameter<std::string>("param1", "Default value", "This is the description")
   .addParameter("param2", 10, "This is an int")
   .addParameter("param3", 11.1, "This is a double")
   .build();

  ROS_ASSERT( std::string("Default value") == config.get<std::string>("param1") );
  ROS_ASSERT( 10 == config.get<int>("param2") );
  ROS_ASSERT( 11.1 == config.get<double>("param3") );

  // The services and the publisher can be configured in a different CallbackQueue.
  // In this example I use an async spinner to execute it in a different thread and
  // be able to call the services from this node.
  ros::NodeHandle n("~");
  ros::CallbackQueue queue;
  n.setCallbackQueue(&queue);
  ros::AsyncSpinner spinner(1, &queue);
  spinner.start();
  ConfigurationServer confServer(n, set_configuration_callback);
  confServer.publish(config.getConfig());

  {
    // The access to the configuration of other node can be wrapped in a few classes
    // to make easier to the user this task. This is not done in this example.
    dynamic_config::GetConfig srv;
    ROS_ASSERT( ros::service::call("~get_server", srv) );
    Configuration config(srv.response.config);
    ROS_ASSERT( std::string("Default value") == config.get<std::string>("param1") );
    ROS_ASSERT( 10 == config.get<int>("param2") );
    ROS_ASSERT( 11.1 == config.get<double>("param3") );
  }

  return 0;
}