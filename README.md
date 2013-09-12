# Dynamic Config

Prototype repository for the new roscpp_param_api

Dynamic Config provides methods to change and query a node configuration. A configuration is a set of parameters that define the behaviour of the node. Nodes can change and query other node's configuration as well as be notified when a change is made to the configuration.


## Building

```bash
$ source /opt/ros/groovy/setup.bash
$ mkdir build
$ cd build
$ cmake ..
$ make
$ source ./devel/setup.bash
```

## Running a demo

Demo running three nodes. This is an example of simple usage of the dynamic_config API.

Start the demo to see the example working. The demo.launch file launches three nodes.

```bash
$ roslaunch dynamic_config demo.launch
```

### Reconfigurable node (server)

```c++
#include <ros/ros.h>
#include "dynamic_config/dynamic_config.h"

// Returns true if the number is even, false otherwise
bool accept_even_numbers(const gsoc::configuration::Configuration& conf) {
  return conf.get<int>("p2")%2 == 0;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "reconfigurable_node");

  namespace config = gsoc::configuration;

  // Create the configuration.
  config::Configuration conf = config::make_builder()
    .addParameter<std::string>("p1", "Hello")
    .addParameter("p2", 100)
    .build();

  // Start configuration server
  ros::NodeHandle n("~");
  config::ConfigurationServer configSrv(n, conf, accept_even_numbers);

  ros::spin();
  return 0;
}
```

A few ROS services and a topic are created to provide the reconfiguration functionalities.

```bash
$ rosservice list /reconfigurable_node | grep -v logger
/reconfigurable_node/get_conf
/reconfigurable_node/set_conf

$ rostopic list /reconfigurable_node
/reconfigurable_node/conf
```

You shouldn't call this services manually!! Unstead use the dynamic_config API.

### Configuation client

The client reconfigures the *reconfigurable_node* every two seconds.

```c++
#include <ros/ros.h>
#include "dynamic_config/dynamic_config.h"

int counter = 0;

void timerCallback(const ros::TimerEvent&, gsoc::configuration::ConfigurationClient& client) {
  // Request the configuration
  gsoc::configuration::Configuration conf = client.configuration();

  // Set new value for param p2
  conf.put("p2", ++counter);

  // Reconfigure the server with the new configuration
  if (client.reconfigure(conf))
    ROS_INFO("Reconfiguration succeed!");
  else
    ROS_INFO("Reconfiguration failed");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "client_node");

  namespace config = gsoc::configuration;

  // Start the configuration client for the reconfigurable node
  ros::NodeHandle n("reconfigurable_node");
  config::ConfigurationClient client(n);

  ros::Timer timer = n.createTimer(ros::Duration(2.), boost::bind(timerCallback, _1, client));

  ros::spin();
  return 0;
}
```

### Configuration listener

This nodes is notified every time the configuration changes and prints the new configuration.

```c++
#include <ros/ros.h>
#include "dynamic_config/dynamic_config.h"

// Print parameter value independently of the type
struct PrintParameter { 
  template <typename T>
  void operator()(std::pair<std::string,T> pair) const {
    ROS_INFO_STREAM("  * " << pair.first << " = " << pair.second);
  }
};

// Configuration listener callback
void configuration_listener(const gsoc::configuration::Configuration& conf) {
  ROS_INFO("New configuration arrived");
  conf.applyAll(PrintParameter());
  ROS_INFO("-------------------------");  
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "listener_node");

  namespace config = gsoc::configuration;

  // Start listener on reconfigurable_node
  ros::NodeHandle n("reconfigurable_node");
  config::ConfigurationListener listener(n, configuration_listener);

  ros::spin();

  return 0;
}
```

## API usage

This is a more detailed description of how to use the API.

### ConfigurationBuilder class

```c++
// make_builder creates a builder in the namespace specified.
// If no namespace is specified, it uses the private namespace
// "~". 
config::ConfigurationBuilder builder = config::make_builder();

// Add parameter p1 with value 100
builder.addParameter("p1", 100)

// Add parameter with the value in parameter ~p2 of the Parameter Server,
// if the parameter doesn't exist the default value is used.
builder.addParameter("p2", "p2", std::string("default_value"));

// Build the configuration
Configuration conf = builder.build();
```

### Configuration class

```c++
// Create an empty configuration
Configuration conf;

// Put a few parameters in conf
conf.put("p1", 100);
conf.put<std::string>("p2", "Hello World!");

// Get the value of the parameters. If the parameter doesn't exist
// or the type is wrong an error message is shown and an undefined
// value is returned.
int p1 = conf.get<int>("p1");
std::string p2 = conf.get<std::string>("p2");

// Check the existance of a parameter
bool exist = conf.has("p1");
bool no_exist = conf.has("no");

// Check type of a parameter
bool right_type = conf.isType<int>("p1");
bool wrong_type = conf.isType<double>("p1");

// Check size
int size = conf.size();

// Parameters names
std::vector<std::string> names(size);
conf.names(names.begin());

// Apply a function to a parameter. The struct passed must match
// all available types for a parameter. The operator() methods must
// be const
struct PrintParameter { 
  template <typename T>
  void operator()(std::pair<std::string,T> pair) const {
    ROS_INFO_STREAM("  * " << pair.first << " = " << pair.second);
  }
};

struct ToString {
  template <typename T>
  std::string operator()(std::pair<std::string,T> pair) const {
    std::stringstream ss;
    ss << pair.second;
    return ss.str();
  }
};

conf.apply("p1", PrintParameter());
int p1 = conf.apply<int>("p1", ToString());

// Apply a function to all the parameters
conf.applyAll(PrintParameter());
std::vector<std::string> strings(size);
conf.apply(ToStrings(), strings.begin());
```

### ConfiguratonServer class

```c++
// Configuration server callback
bool check_conf(const gsoc::configuration::Configuration& conf) {
  // Check something in the new configuration
  return true; // to accept or false to reject
}

// Start a configuration server in the private namespace.
ros::NodeHandle nh("~");
config::ConfigurationServer server(nh, conf, check_conf);

// Always true configuration server
config::ConfigurationServer server(nh, conf);

// Always false configuration server
config::ConfigurationServer server(nh, conf, config::deny_all);

// Get current configuration
Configuration current = server.configuration();

// Set a new configuration. The new configuration must match names and
// types of the parameters.
bool success = server.reconfigure(newConf);
```

### ConfigurationClient class

```c++
// Start a configuration client for node "/server"
ros::NodeHandle nh("server");
config::ConfigurationClient client(nh);

// Get current configuration
Configuration current  = client.configuration();

// Reconfigure the server
bool success = client.reconfigure(newConf);
```

### ConfigurationListener class

```c++
// Configuration listener callback
void configuration_listener(const gsoc::configuration::Configuration& conf) {
  ROS_INFO("New configuration arrived");
}

// Start a configuration listener
ros::NodeHandle nh("server");
config::ConfigurationListener listener(nh, configuration_listener);
```

## Design decisions

### Global/Local parameters

One of the most important decisions was to use global or local parameters. Finally, local parameters have been chosen for many reasons.

A robot control system usually comprises many nodes. These nodes operate at a fine-grained scale and they work together to make more complex tasks. Nodes should be independent and they should do one thing (and do it well). With the design of this component I have tried to improve this features.

When possible local parameters should be used rather than globals because they enhance encapsulation and independence of nodes. But sometimes global parameters are useful. The current Parameter Server is used to initialize values in the configuration.

Local parameters live along with a node. If the node dies, for whatever reason, the configuration dies as well. Parameters has a name and a value.

### Public/Private parameters

All parameters in a configuration are public. Having private parameters in a configuration is only useful for debug purposes. To keep the API simpler private parameters have been omitted.

Parameters in a configuration can be changed at any time. 

### Static/Dynamic parameters

All parameters in a configuration are dynamic. The API doesn't support static parameters. The behaviour of a static parameter can be simulated forbidding manually the change in the server callback code.

### Parameter grouping

Parameters are grouped into configurations. Nodes can have just one configuration. This decision was made to strengthen the single responsibility principle. Nodes should do one thing. If a node needs two or more configurations, maybe that is a sign that the node is doing too much.

### On change notifications

Nodes can request a change at any moment. The user provides a callback which is called every time a change is requested. Only the server node decides if a change is valid or not, again to encourage encapsulation.

The requester gets a true if the new configuration is accepted or a false if not, or the communication with the reconfigurable server fails.

### On update notifications

Nodes can listen to changes on a configuration. The update notifications rely over ROS topics, so there's no guarantee that the change is received by all listeners. I finally took this decision because topics are widely used in ROS systems despite of the non-guarantee delivery mechanism and because using services to do so is quite inefficient and difficult to manage.

### External configuration

External configuration is needed to document a little bit the parameters. Using the wiki or the README.md file from bitbucket are possible solutions.