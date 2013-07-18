#include <ros/ros.h>
#include <std_msgs/String.h>

#include <ros_parameter/ros_parameter.h>

ros::Publisher pub;

// Right now it returns void, but you could also have a function which returns a
// std::map<string, bool> which maps parameter names to True if the change is
// accepted, False if it is rejected This allows the user's code to reject
// the proposed parameter change. You could imagine that you do the change
// acceptance based on state of pg after the function returns, i.e. if the
// parameter values are unchanged by the callback, then the user has accepted
// them. But this would prevent the user from rejecting a value which received
// an update, yet was not changed.
void on_group_change(const ros_parameter::ParameterGroup &pg, 
                     const std::map<std::string, ros_parameter::ParameterGroup::DataType> &newData,
                     std::map<std::string, bool> &changed)
{
    if (changed["~msg1"] && changed["~msg2"]) {
        ROS_INFO("Both changed.");
    } else if (changed["~msg1"]) {
        ROS_INFO("~msg1 changed");
    } else if (changed["~msg2"]) {
        ROS_INFO("~msg2 changed");
    } else {
        ROS_INFO("Neither changed? This shouldn't happen.");
    }
    if (changed["~msg1"] || changed["~msg2"]) {
        std_msgs::String msg;
        // I don't really like this part of the API, see if you can come up with something better
        std::string msg1;
        pg.get_data("~msg1", msg1);
        std::string msg2;
        pg.get_data("~msg2", msg2);
        msg.data = "Message: " + msg1 + " " + msg2 + "!";
        ROS_INFO_STREAM(msg.data);
        pub.publish(msg);
    }
}

template <typename T>
bool on_parameter_change(const ros_parameter::Parameter<T>& param, const T& data)
{
  ROS_INFO_STREAM(param.name() << " changed " << data);
  return true;
}

template <typename T>
bool on_parameter_change_false(const ros_parameter::Parameter<T>& param, const T& data)
{
  return false;
}

template <typename T>
void on_parameter_update(ros_parameter::Parameter<T>& param)
{
  ROS_INFO_STREAM(param.name() << " updated " << param.data());
}

void timer_callback(const ros::TimerEvent &event,
                    ros_parameter::Parameter<int> int1,
                    ros_parameter::Parameter<int> int2,
                    ros_parameter::ParameterGroup pg)
{
  int i = int1.data();
  ++i;
  if (!int1.data(i))
    ROS_INFO_STREAM("Can't change " << int1.name());

  if (!int2.data(0))
    ROS_INFO_STREAM("Can't change " << int2.name());

  ros_parameter::Parameter<std::string> msg1 = pg.get<std::string>("~msg1");
  msg1.data("Hello");

  ros_parameter::Parameter<std::string> msg2 = pg.get<std::string>("~msg2");
  msg2.data("World!!");

  ROS_INFO("----------");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_node");

  ros::NodeHandle n;

   pub = n.advertise<std_msgs::String>("output", 1000);

  // I like this simplified parameter templatization, we should hide the caching options
  // It might even be an option to drop the templating all together and do inheritance
  ros_parameter::Parameter<std::string> msg1("~msg1", "Hello");
  // If I wanted to make msg1 static, I would like to do it like this:
  //   msg1.make_static();
  // Or using a subclass:
  //   ros_parameter::StaticParameter<std::string> msg1(...);
  ros_parameter::Parameter<std::string> msg2("~msg2", "World");

  ros_parameter::ParameterGroup pg;
  // This is nice because the templating of add_parameter is inferred
  pg.add_parameter(msg1);
  pg.add_parameter(msg2);

  // There may be a distinction between on change and on update
  pg.on_change(on_group_change);

  // Handle standalone parameter 
  ros_parameter::Parameter<int> int1("~int1", 0);
  int1.on_change(on_parameter_change<int>);
  int1.on_update(on_parameter_update<int>);

  // Handle standalone parameter which doesn't change
  ros_parameter::Parameter<int> int2("~int2", 0);
  int2.on_change(on_parameter_change_false<int>);
  int2.on_update(on_parameter_update<int>); // Should never be called

  ros::Timer timer = n.createTimer(
  ros::Duration(1),
  boost::bind(timer_callback, _1, int1, int2, pg),
  false);
  timer.start();  

  ros::spin();

  return 0;
}
