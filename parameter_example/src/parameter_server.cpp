#include <iostream>
#include "ros/ros.h"
#include "ros/gsoc/service_param_transport.h"

#include "parameter_example/SetParam.h"

namespace ros {

  namespace gsoc {

    class ParamContainer 
    {
    public:
      ParamContainer() {};
      ~ParamContainer() {};
      void init() {};
    };

    class MyServiceParamTransport
    {
    public:
      MyServiceParamTransport()
	: _nodeHandle("~")
      {}

      ~MyServiceParamTransport()
      {}

      void init()
      {
	_setParamService = _nodeHandle.advertiseService("set_param", &MyServiceParamTransport::setParamService, this);
      }

    private:

      bool setParamService(parameter_example::SetParam::Request  &req,
			   parameter_example::SetParam::Response &res)
      {

      }

      ros::NodeHandle _nodeHandle;
      ros::ServiceServer _setParamService;
    };

    template < class ParamTransport, class ParamWarehouse >
    class ParameterServer
      : public ParamTransport
      , public ParamWarehouse
    {
    public:
      ParameterServer()
	: ParamTransport()
	, ParamWarehouse()
      {}

      ~ParameterServer()
      {}

      void start()
      {
	ParamTransport::init();
	ParamWarehouse::init();
      }

    private:
    };

  } // gsoc

} // ros


int main(int argc, char **argv)
{
  using namespace ros::gsoc;
  ros::init(argc, argv, "parameter_server");
  ParameterServer<MyServiceParamTransport, ParamContainer> paramServer;
  paramServer.start();
  ros::spin();
}

