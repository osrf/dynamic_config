#include <iostream>
#include <map>
#include "ros/ros.h"

#include "parameter_example/SetParam.h"
#include "parameter_example/GetParam.h"
#include "parameter_example/HasParam.h"


namespace parameter_example {

typedef std::map<std::string, std::vector<uint8_t> > ParamContainer;

bool setParamService(parameter_example::SetParam::Request  &req,
		     parameter_example::SetParam::Response &res,
		     ParamContainer *params)
{
  ROS_INFO_STREAM("Setting value for param '" << req.name << "'");
  std::vector<uint8_t>& vec = (*params)[req.name];
  vec.resize(req.value.size());
  std::copy(req.value.begin(), req.value.end(), vec.begin());
  return true;
}

bool getParamService(parameter_example::GetParam::Request  &req,
		     parameter_example::GetParam::Response &res,
		     ParamContainer *params)
{
  ParamContainer::iterator it = params->find(req.name);
  if (it == params->end()) {
    ROS_INFO_STREAM("Param '" << req.name << "' doesn't exist");
    return false;
  }
  ROS_INFO_STREAM("Returning param '" << req.name << "'");
  res.value.resize(it->second.size());
  std::copy(it->second.begin(), it->second.end(), res.value.begin());
  return true; 
}

bool hasParamService(parameter_example::HasParam::Request  &req,
		     parameter_example::HasParam::Response &res,
		     ParamContainer *params)
{
  res.hasParam = (params->find(req.name) != params->end());
  ROS_INFO_STREAM("Param '" << req.name << "' exists? " << res.hasParam);
  return true;
}

class ServiceServers
{
public:
  ServiceServers() : _n("~") {}
  ~ServiceServers() {}; 
  
  template <class MReq, class MRes>
  void advertise(const std::string& srv, bool(*srv_func)(MReq&, MRes&, ParamContainer*))
  {
    _srvs.push_back(_n.advertiseService(srv, bindFunction(srv_func)));
  }

private:

  template <class MReq, class MRes>
  boost::function<bool(MReq&, MRes&)>
  bindFunction( bool(*srv_func)(MReq&, MRes&, ParamContainer*))
  {
    return boost::bind(srv_func, _1, _2, &_params);
  }

  ros::NodeHandle _n;
  std::list<ros::ServiceServer> _srvs;
  ParamContainer _params;
};

} // parameter_example


int main(int argc, char **argv)
{
  using namespace parameter_example;
  ros::init(argc, argv, "parameter_server");

  ServiceServers services;
  services.advertise("set_param", setParamService);
  services.advertise("get_param", getParamService);
  services.advertise("has_param", hasParamService);

  ros::spin();
}

