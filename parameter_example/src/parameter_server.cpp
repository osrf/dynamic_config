#include <iostream>
#include <map>
#include "ros/ros.h"

#include "parameter_example/SetParam.h"
#include "parameter_example/GetParam.h"
#include "parameter_example/HasParam.h"
#include "parameter_example/DeleteParam.h"
#include "parameter_example/SearchParam.h"
#include "parameter_example/SubscribeParam.h"
#include "parameter_example/UnsubscribeParam.h"
#include "parameter_example/ParamEvent.h"


namespace parameter_example {

  namespace {

    template <class From, class To>
    void copyVector(const From& from, To& to)
    {
      to.resize(from.size());
      std::copy(from.begin(), from.end(), to.begin());
    }

  } // anonymous

class SubscribersManager
{
  public:
    SubscribersManager() {};
    ~SubscribersManager() {};

    void add(const std::string& name, const std::string& service)
    {
      ROS_INFO_STREAM("Adding " << name << " - " << service << " to subs");
      subscribers[name].push_back(service);
    }

    void remove(const std::string& name, const std::string& service)
    {
      subscribers[name].remove(service);
    };

    void notifyParamCreated(const std::string& name, 
                            const std::vector<uint8_t> value)
    {
      notify(name, 0, value);
    };

    void notifyParamUpdated(const std::string& name, 
                            const std::vector<uint8_t> value) 
    {
      notify(name, 1, value);
    };

    void notifyParamDeleted(const std::string& name, 
                            const std::vector<uint8_t> value)
    {
      notify(name, 2, value);
    };

  private:

    void notify(const std::string& name, int paramEvent,
                const std::vector<uint8_t> value)
    {
      std::list<std::string>::iterator it = subscribers[name].begin();
      std::list<std::string>::iterator end = subscribers[name].end();
      for (; it!=end; ++it) {
        parameter_example::ParamEvent srv;
        srv.request.name = name;
        srv.request.event = 0; // Created event
        copyVector(value, srv.request.value);
        ROS_INFO_STREAM("Notifying to " << *it);
        ros::service::call(*it, srv);
      }
    }

   std::map<std::string, std::list<std::string> > subscribers;
};

  namespace {  // private namespace

   typedef std::map<std::string, std::vector<uint8_t> > ParamContainer;

    ParamContainer params;

    SubscribersManager subsManager;

  } // anonymous


bool setParamService(parameter_example::SetParam::Request  &req,
		     parameter_example::SetParam::Response &res)
{
  ROS_INFO_STREAM("Setting value for param '" << req.name << "'");

  bool paramDidNotExist = params.find(req.name) == params.end();
  copyVector(req.value, params[req.name]);
  if (paramDidNotExist) 
    subsManager.notifyParamCreated(req.name, req.value);
  else
    subsManager.notifyParamUpdated(req.name, req.value);
  return true;
}

bool getParamService(parameter_example::GetParam::Request  &req,
		     parameter_example::GetParam::Response &res)
{
  ParamContainer::iterator it = params.find(req.name);
  if (it == params.end()) {
    ROS_INFO_STREAM("Param '" << req.name << "' doesn't exist");
    return false;
  }
  ROS_INFO_STREAM("Returning param '" << req.name << "'");
  res.value.resize(it->second.size());
  std::copy(it->second.begin(), it->second.end(), res.value.begin());
  return true; 
}

bool hasParamService(parameter_example::HasParam::Request  &req,
		     parameter_example::HasParam::Response &res)
{
  res.hasParam = (params.find(req.name) != params.end());
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
  std::vector<uint8_t> value(params[req.name]);
  params.erase(req.name);
  subsManager.notifyParamDeleted(req.name, value);
  return true;
}

bool subscribeParamService(parameter_example::SubscribeParam::Request  &req,
                           parameter_example::SubscribeParam::Response &res)
{
  subsManager.add(req.name, req.service);
  return true;
}

bool unsubscribeParamService(parameter_example::UnsubscribeParam::Request  &req,
                             parameter_example::UnsubscribeParam::Response &res)
{
  subsManager.remove(req.name, req.service);
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


  ros::spin();
}

