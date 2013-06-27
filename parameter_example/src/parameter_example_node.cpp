#include "ros/ros.h"
#include "std_msgs/String.h"

#include "ros/gsoc/parameter.h"

#include <boost/bind.hpp>

void timer_callback(
  const ros::TimerEvent &event,
  const ros::Publisher &pub,
  const ros::gsoc::Parameter<std::string> &message)
{
  std_msgs::String msg;
  // If the message is dynamically updated it should take affect here
  // msg.data = message.getData();
  //ROS_INFO_STREAM("Sending '" << msg.data << "'");
  //pub.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "foo");
  ros::NodeHandle n;
  ros::gsoc::ParameterInterface pi;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  // Ideally I should be abled to dynamically update this parameter now
  ros::gsoc::Parameter<std::string> message = pi.createParameter<std::string>(
    "/foo/message",
    "Hello World");

  ros::Timer timer = n.createTimer(
    ros::Duration(1),
    boost::bind(timer_callback, _1, chatter_pub, message),
    false);
  timer.start();

  ros::gsoc::Parameter<int> myParam = pi.createParameter<int>("/myint", "fakeCallback", 0);

  ros::spin();

  return 0;
}
