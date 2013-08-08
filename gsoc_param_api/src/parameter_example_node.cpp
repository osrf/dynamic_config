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
#include "gsoc/param_server_proxy.h"
#include "gsoc/param_server.h" 

namespace gsoc {

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

    template < typename T >
    void deserialize(std::vector<uint8_t>& data, T& output)
    {
      ros::serialization::IStream istream(&data[0], data.size());
      ros::serialization::Serializer<T>::read(istream, output);
    }

    void deserialize(std::vector<uint8_t>& data, std::vector<uint8_t>& output)
    { output = data; }

  } // serialization

  template <typename T>
  class Parameter {
  public:
    Parameter(const std::string& name, const T& data, param::ParamServerProxyPtr serverProxyPtr)
    : name_(name)
    , data_(data)
    , serverProxyPtr_(serverProxyPtr)
    { }

    ~Parameter()
    { }

    const std::string& name() const
    { return name_; }

    const T& data() const
    { return data_; }

    bool commit() const
    {
      std::vector<uint8_t> raw;
      serialization::serialize(data_, raw);
      return serverProxyPtr_->set(name_, raw);
    }

    void set(const T& data)
    { data_ = data; }

  private:
    std::string name_;
    T data_;
    param::ParamServerProxyPtr serverProxyPtr_;
  };

  template <typename T>
  class ParameterListener {
  public:
    typedef boost::function<void (const Parameter<T>&, const Parameter<T>&)> Callback;

    ParameterListener(const std::string& name, Callback cb, param::ParamServerProxyPtr serverProxyPtr)
    : name_(name)
    , callback_(cb)
    , serverProxyPtr_(serverProxyPtr)
    {
      serverProxyPtr_->addUpdater(name_, boost::bind(&ParameterListener::raw_callback, this, _1, _2));
    }

    ~ParameterListener()
    { shutdown(); }

    void shutdown() 
    {
      serverProxyPtr_->removeUpdater(name_);
      callback_ = NULL;
    }

  private:
    void raw_callback(const std::vector<uint8_t>& old, const std::vector<uint8_t>& actual)
    {
      std::vector<uint8_t> old_noConst(old);
      std::vector<uint8_t> actual_noConst(actual);
      T oldData, actualData;
      serialization::deserialize(old_noConst, oldData);  
      serialization::deserialize(actual_noConst, actualData);  
      callback_(Parameter<T>(name_, oldData, serverProxyPtr_),
                Parameter<T>(name_, actualData, serverProxyPtr_));
    }

    std::string name_;
    Callback callback_;
    param::ParamServerProxyPtr serverProxyPtr_;
  };

  class ParameterGroup {
  public:
    ParameterGroup(const std::string& namespc, param::ParamServerProxyPtr serverProxyPtr)
    : namespc_(namespc)
    , serverProxyPtr_(serverProxyPtr)
    { }

    ~ParameterGroup()
    { }

    template <typename T>
    Parameter<T> createParameter(const std::string& name)
    {
      T default_data;
      return createParameter(name, default_data);
    }

    template <typename T>
    Parameter<T> createParameter(const std::string& name, const T& default_data)
    {
      std::string fullName = namespc_ + "/" + name;
      std::vector<uint8_t> ser;
      if ( serverProxyPtr_->get(fullName, ser) ) {
        T data;
        serialization::deserialize(ser, data);  
        return Parameter<T>(fullName, data, serverProxyPtr_);
      } else
        return Parameter<T>(fullName, default_data, serverProxyPtr_);
    }

    template <typename T>
    ParameterListener<T> createParameterListener(const std::string& name, boost::function<void (const Parameter<T>&, const Parameter<T>&)> cb)
    {
      std::string fullName = namespc_ + "/" + name;
      return ParameterListener<T>(fullName, cb, serverProxyPtr_);
    }

  private:
    std::string namespc_;
    param::ParamServerProxyPtr serverProxyPtr_;
  };

  class ParamHandle {
  public:
    ParamHandle(const std::string& namespc)
    : serverProxyPtr_(new param::ParamServerProxy(namespc))
    { }

    ~ParamHandle()
    { }

    ParameterGroup createParameterGroup(const std::string& namespc) {
      return ParameterGroup(namespc, serverProxyPtr_);
    }

  private:
    param::ParamServerProxyPtr serverProxyPtr_;
  };

} // gsoc

bool parameter_listener_callback_called = false;
void parameter_listener_callback(const gsoc::Parameter<std::string>& old, 
                                 const gsoc::Parameter<std::string>& actual)
{
  parameter_listener_callback_called = true;
  ROS_ASSERT( "old string" == old.data() );
  ROS_ASSERT( "actual string" == actual.data() );
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "parameter_example_node");

  gsoc::ParamHandle ha("/node_a");
  gsoc::ParamHandle hb("/node_b");

  gsoc::param_server::init("param_server");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  {
    // Node A creates parameter /namespace_a/param1
    gsoc::ParameterGroup groupA = ha.createParameterGroup("/namespace_a");
    gsoc::Parameter<std::string> paramA = groupA.createParameter("param1", std::string("default string"));

    // paramA name should be /namespace_a/param1
    ROS_ASSERT( "/namespace_a/param1" == paramA.name() );
 
    // paramA data should be "default string"
    ROS_ASSERT( "default string" == paramA.data() );

    // Set & Commit paramA to server
    paramA.set("string after commit");
    ROS_ASSERT( paramA.commit() );

    // A second parameter should return the same value "string after commit"
    {
      gsoc::Parameter<std::string> paramA_bis = groupA.createParameter("param1", std::string("another string"));
      ROS_ASSERT( "string after commit" == paramA_bis.data() );
    
      gsoc::Parameter<std::string> paramA_bis2 = groupA.createParameter<std::string>("param1");
      ROS_ASSERT( "string after commit" == paramA_bis2.data() );      
    }

    // Node B creates a listener
    paramA.set("old string");
    paramA.commit();
    gsoc::ParameterGroup groupB = hb.createParameterGroup("/namespace_a");
    gsoc::ParameterListener<std::string> listener 
      = groupB.createParameterListener<std::string>("param1", parameter_listener_callback);
    paramA.set("actual string");
    ROS_ASSERT( paramA.commit() );
    ROS_ASSERT( parameter_listener_callback_called );
  }

  ROS_INFO("Test finished correctly");
}