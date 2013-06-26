#include "ros/ros.h"
#include "ros/gsoc/parameter.h"
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

template <class T>
void setParameterInServer(const std::string& name,
			  const T& value)
{
  ros::gsoc::ParameterInterface pi;
  ros::gsoc::Parameter<T, ros::gsoc::NonCachePolicy> param = 
    pi.createParameter<T, ros::gsoc::NonCachePolicy>(name);
  param.setData(value);
}

void setParamArg(const std::string& name,
		 const std::string& value)
{
  // Int
  if (boost::regex_match(value, boost::regex("-?[0-9]+"))) {
    setParameterInServer(name, boost::lexical_cast<int>(value));

  // Float
  } else if (boost::regex_match(value, boost::regex("-?[0-9]*\\.[0-9]+f"))) {
    std::string substr(value.begin(), --value.end());
    setParameterInServer(name, (float)boost::lexical_cast<double>(substr));
  
  // Double
  } else if (boost::regex_match(value, boost::regex("-?[0-9]*\\.[0-9]+"))) {
    setParameterInServer(name, boost::lexical_cast<double>(value));

  // Boolean
  } else if (boost::regex_match(value, boost::regex("true|false"))) {
    bool aBool = (value == "true");
    setParameterInServer(name, aBool);

  // String. Default option.
  } else {
      setParameterInServer(name, value);
  }
}

template <class T>
void printParameter(const std::string& name)
{
  ros::gsoc::ParameterInterface pi;
  ros::gsoc::Parameter<T, ros::gsoc::NonCachePolicy> param = 
    pi.createParameter<T, ros::gsoc::NonCachePolicy>(name);
  if (param.exist()) {
    ROS_INFO_STREAM(param.getData());
  } else {
    ROS_INFO_STREAM("Parameter [" << name << "] is not set");
  }
}

void getParamArg(const std::string& name, 
		 const std::string& type)
{
  if      (type == "string") printParameter<std::string>(name);
  else if (type == "int"   ) printParameter<int>(name);
  else if (type == "float" ) printParameter<float>(name);
  else if (type == "double") printParameter<double>(name);
  else if (type == "bool"  ) printParameter<bool>(name);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosparam_node");
  ros::NodeHandle n;

  std::string arg1(argv[1]);

  if ("set" == arg1) {
    // Check number of arguments
    if (argc < 4) {
      ROS_INFO("Bad number of arguments for arg set");
      return -1;
    }
    setParamArg(argv[2], argv[3]);

  } else if ("get" == arg1) {
    if (argc < 4) {
      ROS_INFO("Bad number of arguments for arg set");
      return -1;
    }
    getParamArg(argv[2], argv[3]);
  }

}
