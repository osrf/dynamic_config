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

#include <fstream>

#include <ros/ros.h>
#include <ros/this_node.h>
#include <ros/callback_queue.h>

#include <boost/variant.hpp>

#include <dynamic_config/GetConf.h>
#include <dynamic_config/SetConf.h> 

#include "dynamic_config/dynamic_config.h"
#include "gsoc/configuration/msg_handler.h"
// #include "gsoc/configuration/configuration.h"
// #include "gsoc/configuration/configuration_builder.h"
// #include "gsoc/configuration/configuration_server.h"
// #include "gsoc/configuration/configuration_client.h"

struct ToString {

  template <typename T>
  std::string operator()(std::pair<std::string,T> pair) const {
    std::string out = pair.first;
    out += "=";
    out += toString(pair.second);
    return out;
  }

  template <typename T>
  std::string toString(const T& t) const {
    try {
      return boost::lexical_cast<std::string>(t);
    } catch (boost::bad_lexical_cast &) {
      return "unknown";
    }
  }

  std::string toString(const std::string& s) const {
    std::string out = "\"";
    out += s;
    out += "\"";
    return out;
  }

  std::string toString(float f) const {
    std::string out = boost::lexical_cast<std::string>(f);
    out += "f";
    return out;
  }

  std::string toString(bool b) const {
    return b ? "true" : "false";
  }

  std::string toString(long l) const {
    std::string out = boost::lexical_cast<std::string>(l);
    out += "l";
    return out;
  }

};

// This is struct is to do introspection of the configuration.
// Methods must be const. All possible types of the parameter must
// have a function (For the example I just have string and int). A
// template function can also be used.
struct PrintConfiguration {
  template <typename T>
  void operator()(std::pair<std::string,T> pair) const {
    ToString op;
    ROS_INFO_STREAM("Param " << op(pair));
  }
};

bool accept_configuration(gsoc::configuration::Configuration& conf) {
  return true;
}

void configuration_listener(gsoc::configuration::Configuration& conf) {
  // The client can do introspection in the configuration
  conf.applyAll(PrintConfiguration());
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "config_prototype_node");

  // Server is executed in a different queue to simulate
  // it is a different node
  ros::NodeHandle srvRosHandle("server");
  ros::CallbackQueue srvQueue;
  srvRosHandle.setCallbackQueue(&srvQueue);
  ros::AsyncSpinner srvSpinner(1, &srvQueue);
  srvSpinner.start();

  // Server starts the configuration server with a configuration
  // The values of the configuration can be changed, but not the structure
  // i.e., the parameters name and type
  config::Configuration conf = config::make_builder()
    .addParameter("p1", std::string("hola holita"))
    .addParameter("p2", 100)
    .addParameter("p3", true)
    .addParameter("p4", 10.5f)
    .addParameter("p5", 10.6)
    .addParameter("p6", long(1000))
    .build();

  std::ofstream ofs("/tmp/miconf.cfg");
  std::ostream_iterator<std::string> it(ofs, "\n");
  conf.applyAll<std::string>(ToString(), it);
  ofs.close();

  std::ifstream ifs("/tmp/miconf.cfg");
  config::Configuration conf_bis = config::make_builder()
    .addParameters(ifs)
    .build();
  ifs.close();
  ROS_ASSERT(conf == conf_bis);

  config::ConfigurationServer configSrv(srvRosHandle, conf);

  // Listener of a configuration
  ros::NodeHandle n("server");
  config::ConfigurationListener listener(n, configuration_listener);

  // A client request the configuration. Both configs are equal.
  config::ConfigurationClient client(n);
  config::Configuration conf2 = client.configuration();
  ROS_ASSERT( conf == conf2 );

  // The client request a reconfiguration
  conf2.put("p1", std::string("bye bye"));
  conf2.put("p2", 200);
  ROS_ASSERT( client.reconfigure(conf2) );
  ROS_ASSERT( configSrv.configuration() == conf2 );

  // Spin to call listener queue
  ROS_INFO("Press C^c to finish the test");
  ros::spin();

  ROS_INFO("Test finished!!");
  return 0;
}