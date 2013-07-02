#ifndef PARAMETER_MANAGER_H
#define PARAMETER_MANAGER_H

#include <iostream>

#include <boost/ptr_container/ptr_map.hpp>
#include <boost/signals.hpp>
#include "ros/ros.h"

#include "parameter_example/ParamEvent.h"
#include "parameter_example/SubscribeParam.h"
#include "parameter_example/UnsubscribeParam.h"


namespace ros {

  namespace gsoc {

class ParameterGateway
{
 public:
  typedef boost::function<void (const std::string&, int)> EventCallback;

  ParameterGateway() {}

  ParameterGateway(EventCallback& cb)
    : cb_(cb)
    {
      srvServer_ = ros::NodeHandle("~").advertiseService
        ("param_event", &ParameterGateway::srvCallback, this);
    }

  void subscribe(const std::string& name)
  {
    parameter_example::SubscribeParam srv;
    srv.request.name = name;
    srv.request.service = srvServer_.getService();
    std::string srv_name("/parameter_server/subscribe_param");
    ros::service::call(srv_name, srv);
  }

  void unsubscribe(const std::string& name)
  {
    parameter_example::UnsubscribeParam srv;
    srv.request.name = name;
    srv.request.service = srvServer_.getService();
    std::string srv_name("/parameter_server/unsubscribe_param");
    if (!ros::service::call(srv_name, srv)) {
      ROS_ERROR_STREAM("Error unsubscribing parameter " << name << " from server");
    } else {
      ROS_INFO_STREAM("Unsubscribe " << name << " succesfully");
    }
  }

  bool srvCallback(parameter_example::ParamEvent::Request  &req,
                   parameter_example::ParamEvent::Response &res)
  {
    cb_(req.name, req.event);
    return true;
  }

 private:
  ros::ServiceServer srvServer_;
  EventCallback cb_;
};


class ParameterManager;
typedef boost::shared_ptr<ParameterManager> ParameterManagerPtr;
ParameterManagerPtr g_parameter_manager;

class ParameterManager
{
 private:
  typedef boost::signal<void (int)> Signal;
  typedef boost::shared_ptr<Signal> SignalPtr;

 public:
  typedef boost::signals::connection ParameterManagerConnection;

  static ParameterManagerPtr instance()
  {
    if (!g_parameter_manager) {
      g_parameter_manager.reset(new ParameterManager);
    }
    return g_parameter_manager;
  }

  ParameterManager()
  : eventCallback_(boost::bind(&ParameterManager::callback, this, _1, _2))
  , paramGtway_(eventCallback_)
  {}

  ~ParameterManager()
  {}

  void callback(const std::string& name, int event)
  {
    (*prxs_[name])(event);
  }

  ParameterManagerConnection connect(const std::string& name, Signal::slot_function_type& func)
  {
    if (prxs_.find(name) == prxs_.end()) {
      prxs_[name] = SignalPtr(new Signal);  
    }
    
    if (prxs_[name]->empty()) {
      paramGtway_.subscribe(name);
    }
   
    return prxs_[name]->connect(func);
  }

  void disconnect(const std::string& name, ParameterManagerConnection& cb)
  {
    cb.disconnect();
    if (prxs_[name]->empty()) {
      std::cerr << "unsubscribing" << std::endl;
      paramGtway_.unsubscribe(name);
    }
  }  

 private:
  std::map<std::string, SignalPtr> prxs_;
  ParameterGateway::EventCallback eventCallback_;
  ParameterGateway paramGtway_;
};

  } // gsoc

} // ros

#endif
