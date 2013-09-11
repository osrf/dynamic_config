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

## Design decisions

### Global/Local parameters

One of the most important decisions was to use global or local parameters. Finally, local parameters have been chosen for many reasons.

A robot control system usually comprises many nodes. These nodes operate at a fine-grained scale and they work together to make more complex tasks. Nodes should be independent and they should do one thing (and do it well). With the design of this component I have tried to improve this features.

When possible local parameters should be used rather than globals because they enhance encapsulation and independence of nodes. But sometimes global parameters are useful. The current Parameter Server is used to initialize values in the configuration.

Local parameters live along with a node. If the node dies, for whatever reason, the configuration dies as well.

A parameter of a configuration is composed by a name and a value. 

```c++
// Create a ConfigurationBuilder using the private namespace
config::Configuration conf = config::make_builder("~")
  // Add parameter with a default value.
  .addParameter("p1", std::string("hello"))
  // Add parameter with the value in parameter ~global_p2 of the Parameter Server,
  // if the parameter doesn't exist the default value is used.
  .addParameter("p2", "global_p2", 100)
  .build();
```

### Public/Private parameters

All parameters in a configuration are public. Having private parameters in a configuration is only useful for debug purposes. To keep the API simpler private parameters have been omitted.

Parameters in a configuration can be changed at any time. 

```c++
// Check if a parameter exist
bool exist = conf.has("p1");

// Check type of a parameter. If the type is incorrect or the 
// parameter doesn't exist it returns false
bool correctType = conf.isType<std::string>("p1");

// Request a parameter. If the parameter has an incorrect type
// or the parameters doesn't exist an error message is shown and
// an undefined value is returned.
std::string p1 = conf.get<std::string>("p1");

// Set a new value for a parameter. If the parameter doesn't exist,
// a new parameter is added to the configuration
conf.put("p1", std::string("world"));
```

### Static/Dynamic parameters

All parameters in a configuration are dynamic. The API doesn't support static parameters. The behaviour of a static parameter can be simulated forbidding manually the change in the server callback code.

### Parameter grouping

Parameters are grouped into configurations. Nodes can have just one configuration. This decision was made to strengthen the single responsibility principle. Nodes should do one thing. If a node needs two or more configurations, maybe that is a sign that the node is done too much.

### On change notifications

Nodes can request a change at any moment. 

```c++
bool accept_config(gsoc::configuration::Configuration& conf) {
  // Check new configuration. Return true to accept or false to reject
  return true;
}

ros::NodeHandle n("~");
// Init configuration server using private namespace with 
// configuration conf. Values of the configuration can be
// change, but neither names or types can be changed.
config::ConfigurationServer server(n, conf, accept_config);

// If all configurations are accepted the server can be launched like this
config::ConfigurationServer accept_all_server(n, conf);

// If all configurations are rejected use
config::ConfigurationServer reject_all_server(n, conf, config::reject_all);
```

```c++
// A client request the configuration.
ros::NodeHandle n("/node");
config::ConfigurationClient client(n);

// Get the configuration of "/node"
config::Configuration conf = client.configuration();

// Change some parameters
conf.put<std::string>("p1", "new value");
conf.put("p2", 300);

// Reconfigure node. Returns true is succed, false otherwise
bool reconfigured = client.reconfigure(conf);
```

```c++
// Prints parameter name and value
struct PrintConfiguration { 
  template <typename T>
  void operator()(std::pair<std::string,T> pair) const {
    ROS_INFO_STREAM("The parameter " << pair.first << " has value " << pair.second);
  }
};

void configuration_listener(gsoc::configuration::Configuration& conf) {
  conf.applyAll(PrintConfiguration());
}

// The callback is called when a change is made
ros::NodeHandle n("/node");
config::ConfigurationListener listener(n, configuration_listener);
```