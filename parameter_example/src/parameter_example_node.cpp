#include "ros/ros.h"
#include "std_msgs/String.h"

#include "ros/gsoc/parameter.h"

#include <boost/bind.hpp>

void timer_callback(
  const ros::TimerEvent &event,
  const ros::Publisher &pub,
  ros::gsoc::Parameter<int> &message)
{
  std_msgs::String msg;
  // If the message is dynamically updated it should take affect here
  // msg.data = message.getData();
  //ROS_INFO_STREAM("Sending '" << msg.data << "'");
  //pub.publish(msg);
  int data = message.getData() + 1;
  message.setData(data);
}

void param_callback(int paramEvent, ros::gsoc::Parameter<int>& param)
{
  ROS_INFO_STREAM(param.getData());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "foo");
  ros::NodeHandle n;
  ros::gsoc::ParameterInterface pi;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  // Ideally I should be abled to dynamically update this parameter now
  // ros::gsoc::Parameter<std::string> message = pi.createParameter<std::string>(
  //   "/foo/message",
  //   "Hello World");


  ros::gsoc::Parameter<int> myParam = pi.createParameter<int>("/myint");
  myParam.setData(1);

  ros::Timer timer = n.createTimer(
    ros::Duration(1),
    boost::bind(timer_callback, _1, chatter_pub, myParam),
    false);
  timer.start();

  boost::function<void (int, ros::gsoc::Parameter<int>&)> f = param_callback;
  ros::gsoc::ParameterServer<int> paramServer 
   = pi.createParameterServer<int>("/myint", f);


  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  return 0;
}
