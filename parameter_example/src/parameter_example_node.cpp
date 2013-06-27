#include "ros/ros.h"
#include "std_msgs/String.h"

#include "ros/gsoc/parameter.h"

#include <boost/bind.hpp>

template < class T >
void timer_callback(
  const ros::TimerEvent &event,
  const ros::Publisher &pub,
  T& message)
  // const ros::gsoc::Parameter<std::string, ros::gsoc::NonCachePolicy> &message)
{
  std_msgs::String msg;
  // If the message is dynamically updated it should take affect here
  msg.data = message.getData();
  ROS_INFO_STREAM("Sending '" << msg.data << "'");
  pub.publish(msg);
}

template < class CachePolicy > 
ros::Timer createTimer(const std::string& name, const std::string& default_value)
{
  // Ideally I should be abled to dynamically update this parameter now
  typedef ros::gsoc::Parameter<std::string,CachePolicy> ParameterType;

  ros::NodeHandle n;
  ros::gsoc::ParameterInterface pi;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ParameterType message = pi.createParameter<std::string, CachePolicy>(
    name,
    default_value);

  ros::Timer timer = n.createTimer(
    ros::Duration(1),
    boost::bind(timer_callback<ParameterType>, _1, chatter_pub, message),
    false);
  timer.start();

  return timer;
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "foo");
  ros::Timer timer1 = createTimer<ros::gsoc::NonCachePolicy>("/foo/message", "My default value");
  // ros::Timer timer2 = createTimer<ros::gsoc::CachePolicy>("/foo/cacheMessage", "My cache default value");
  ros::spin();

  return 0;
}
