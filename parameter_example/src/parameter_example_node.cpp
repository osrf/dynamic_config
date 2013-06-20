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
  msg.data = message.get_data();
  pub.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "foo");
  ros::gsoc::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  // Ideally I should be abled to dynamically update this parameter now
  ros::gsoc::Parameter<std::string> message = n.createParameter<std::string>(
    "~message",
    "Message for me to echo",
    "Hello World");

  ros::Timer timer = n.createTimer(
    ros::Duration(1),
    boost::bind(timer_callback, _1, chatter_pub, message),
    false);
  timer.start();

  ros::spin();

  return 0;
}