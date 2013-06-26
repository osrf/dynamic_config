#include <boost/shared_ptr.hpp>
#include <map>
#include <ros/ros.h>

#include "parameter_example/SetParam.h"
#include "parameter_example/GetParam.h"
#include "parameter_example/HasParam.h"

namespace ros {

  namespace gsoc {

class ParamServerClient
{
 public:
  ParamServerClient() {};
  ~ParamServerClient() {};

  template <typename T>
  void setParam(const std::string& name, const T& value)
  {
    typedef parameter_example::SetParam Srv;
    Srv srv;
    srv.request.name = name;
    serialize(value, srv.request.value);
    getClient<Srv>("/parameter_server/set_param").call(srv);
  }

  template <typename T>
  bool getParam(const std::string& name, T& value)
  {
    typedef parameter_example::GetParam Srv;
    Srv srv;
    srv.request.name = name;
    if ( getClient<Srv>("/parameter_server/get_param").call(srv) ) {
      deserialize(srv.response.value, value);
      return true;
    } else {
      ROS_ERROR_STREAM("Unable to call 'get_param' service");
      return false;
    }
  }

  bool hasParam(const std::string& name)
  {
    typedef parameter_example::HasParam Srv;
    Srv srv;
    srv.request.name = name;
    if ( getClient<Srv>("/parameter_server/has_param").call(srv) ) {
      return srv.response.hasParam;
    } else {
      ROS_ERROR_STREAM("Unable to call 'has_param' service");
      return false;
    }    
  }

 private:
  typedef std::map<std::string, ros::ServiceClient> Clients;

  // Lazy services initialization
  template <class T>
  ros::ServiceClient& getClient(const std::string& srv)
  {
    Clients::iterator it = _clients.find(srv);
    if (it == _clients.end()) {
      it = _clients.insert(make_pair(srv,_n.serviceClient<T>(srv))).first;
    }
    return it->second;
  }

  template < typename T >
  void serialize(const T& data, std::vector<uint8_t>& buffer)
  {
    buffer.resize(ros::serialization::serializationLength(data));
    ros::serialization::OStream ostream(&buffer[0], buffer.size());
    ros::serialization::serialize(ostream, data);
  }

  template < typename T >
  void deserialize(std::vector<uint8_t>& data, T& output)
 {
   ros::serialization::IStream istream(&data[0], data.size());
   ros::serialization::Serializer<T>::read(istream, output);  
 }

  ros::NodeHandle _n;
  std::map<std::string, ros::ServiceClient> _clients;
};

// Every time a node wants to node the value of the
// parameter, it is requested to the param server.
class NonCachePolicy 
{
 protected:
  template < class T >
  bool pullData(const std::string& name, T& data)
  {
    return _client.getParam(name, data);
  }

  // Not sure if this method shoud be const. The state of the 
  // object doesn't change but it does changes in the param
  // server.
  template < class T >
  void pushData(const std::string& name, const T& data)
  {
    _client.setParam(name, data);
  }

  bool hasData(const std::string& name)
  {
    return _client.hasParam(name);
  }

 private:
  ParamServerClient _client;
};

// To keep updated the value of the parameter, this class
// should use the susbcribe method.
class CachePolicy
{
 protected:
  template < class T >
  bool pullData(const std::string& name, T& data) const
  {
    return ros::param::getCached(name, data);
  }

  template < class T >
  void pushData(const std::string& name, const T& data) const
  {
    ros::param::set(name, data);
  }

  bool hasData(const std::string& name)
  {
    return ros::param::has(name);
  }
};

// To get and set the value of the parameter it uses
// the policy specified.
template <class T, class CachePolicy>
class Parameter : public CachePolicy
{
public:
  Parameter(const std::string &name, const T &default_value)
  : _name(name)
  {
    if (!exist()) {
      setData(default_value);
    }
  }

  Parameter(const std::string &name)
  : _name(name)
  { }

  Parameter() {}

  // Copy Constructors
  Parameter(const Parameter &other) : _name(other._name) {}

  ~Parameter() {}

  const std::string& getName()
  {
    return _name;
  }

  T getData()
  {
    // You could do locking here to prevent people from accessing while updating
    // Do an implicit copy otherwise the locking really doesn't do anything
    T value;
    CachePolicy::pullData(_name, value);
    return value;
    }

  void setData(const T &value)
  {
    // Again doing a copy to allow for locking in the future
    CachePolicy::pushData(_name, value);
  }

  bool exist()
  {
    return CachePolicy::hasData(_name);
  }

private:
  std::string _name;

};


namespace param {

  template <class T, class U>
  void get(const std::string &name, Parameter<T,U> &param, const T &default_value)
  {
    param = Parameter<T,U>(name, default_value);
  }

  template <class T, class U>
  void get(const std::string &name, Parameter<T,U> &param) {
    param = Parameter<T,U>(name);
  }
  
}


// This could be integrated into the ros::NodeHandle object once the API is set

class ParameterInterface {
public:
  
  template <class T, class U>
  Parameter<T,U> createParameter(const std::string &name, 
				 const T &default_value) 
  {
    Parameter<T,U> p;
    param::get(name, p, default_value);
    return p;
  }

  template <class T, class U>
  Parameter<T,U> createParameter(const std::string &name) 
  {
    Parameter<T,U> p;
    param::get(name, p);
    return p;
  }

};


  } // gsoc

} // ros

