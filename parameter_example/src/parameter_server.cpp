#include <iostream>
#include <map>
#include "ros/ros.h"

#include "parameter_server/parameters_server.h"
#include "parameter_server/command.h"
#include "parameter_server/commands_queue.h"

#include "parameter_example/SetParam.h"
#include "parameter_example/GetParam.h"
#include "parameter_example/HasParam.h"
#include "parameter_example/DeleteParam.h"
#include "parameter_example/SearchParam.h"
#include "parameter_example/SubscribeParam.h"
#include "parameter_example/UnsubscribeParam.h"
#include "parameter_example/ParamEvent.h"


namespace parameter_example {

  namespace {  // private namespace

    namespace comm = parameter_server::command;
    comm::CommandsQueue<comm::Command> commands;

    typedef std::vector<uint8_t> Value;
    typedef parameter_server::ParametersServer<Value> ParamServer;
    ParamServer paramServer;

    class Callback
    {
    public:

      Callback(const std::string& srv, const std::string& name)
      : service_(srv), name_(name) { }

      bool callback(const ParamServer::Response& res)
      { 
        ROS_INFO_STREAM("Callback!!!\n  Service: " << service_
          << "\n  Param: " << name_ << "\n  Event: " << res.event);

        parameter_example::ParamEvent evt;
        evt.request.name = name_;
        evt.request.event = res.event;

        comm::Command c(
          boost::bind(ros::service::call<parameter_example::ParamEvent>, 
          service_, evt)
        );
        commands.add(c);
        ROS_INFO_STREAM("Commands queue size " << commands.size());

        // if (!ros::service::call(service_, evt)) {
        //   ROS_ERROR_STREAM("Unable to call " << service_);
        // }
      }

    private:
      std::string service_;
      std::string name_;
    };
    typedef boost::shared_ptr<Callback> CallbackPtr;

    typedef std::map<std::string, CallbackPtr> ParamCallbacks;
    typedef std::map<std::string, ParamCallbacks> ServiceCallbacks;
    ServiceCallbacks srvCbs;

  } // anonymous


bool setParamService(parameter_example::SetParam::Request  &req,
		                 parameter_example::SetParam::Response &res)
{
  ROS_INFO_STREAM("Setting value for param '" << req.name << "'");
  paramServer.set(req.name, req.value);
  return true;
}

bool getParamService(parameter_example::GetParam::Request  &req,
		                 parameter_example::GetParam::Response &res)
{
  ROS_INFO_STREAM("Returning param '" << req.name << "'");
  paramServer.get(req.name, res.value);
  return true; 
}

bool hasParamService(parameter_example::HasParam::Request  &req,
		                 parameter_example::HasParam::Response &res)
{
  res.hasParam = paramServer.has(req.name);
  ROS_INFO_STREAM("Param '" << req.name << "' exists? " << res.hasParam);
  return true;
}

bool searchParamService(parameter_example::SearchParam::Request  &req,
		     parameter_example::SearchParam::Response &res)
{
  return true;
}


bool deleteParamService(parameter_example::DeleteParam::Request  &req,
                        parameter_example::DeleteParam::Response &res)
{
  return true;
}

bool subscribeParamService(parameter_example::SubscribeParam::Request  &req,
                           parameter_example::SubscribeParam::Response &res)
{
  CallbackPtr cb(new Callback(req.service, req.name));
  srvCbs[req.service][req.name] = cb;
  paramServer.subscribe(req.name, &Callback::callback, cb);
  return true;
}

bool unsubscribeParamService(parameter_example::UnsubscribeParam::Request  &req,
                             parameter_example::UnsubscribeParam::Response &res)
{
  ROS_INFO_STREAM("Unsubscribing param " << req.name << " of service " << req.service);
  srvCbs[req.service].erase(req.name);
  return true;
}


} // parameter_example


int main(int argc, char **argv)
{
  using namespace parameter_example;
  ros::init(argc, argv, "parameter_server");
  ros::NodeHandle n("~");

  ros::ServiceServer srv1 = n.advertiseService("set_param", setParamService);
  ros::ServiceServer srv2 = n.advertiseService("get_param", getParamService);
  ros::ServiceServer srv3 = n.advertiseService("has_param", hasParamService);
  ros::ServiceServer srv4 = n.advertiseService("search_param", searchParamService);
  ros::ServiceServer srv5 = n.advertiseService("delete_param", deleteParamService);
  ros::ServiceServer srv6 = n.advertiseService("subscribe_param", subscribeParamService);
  ros::ServiceServer srv7 = n.advertiseService("unsubscribe_param", subscribeParamService);

  // ros::MultiThreadedSpinner spinner(4);
  // ros::Rate r(30);
  // while (ros::ok()) {
  //   commands.executeAll();
  //   ros::spinOnce();
  //   r.sleep();
  // }

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Rate r(30);
  while (ros::ok()) {
    commands.executeAll();
    r.sleep();
  }
}
