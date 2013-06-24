#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

namespace ros {

  namespace gsoc {

// Every time a node wants to node the value of the
// parameter, it is requested to the param server.
class NonCachePolicy 
{
 protected:
  template < class T >
  bool pullData(const std::string& name, T& data) const
  {
    return ros::param::get(name, data);
  }

  // Not sure if this method shoud be const. The state of the 
  // object doesn't change but it does changes in the param
  // server.
  template < class T >
  void pushData(const std::string& name, const T& data) const
  {
    ros::param::set(name, data);
  }
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
    if (!ros::param::has(name)) {
      ros::param::set(name, default_value);
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

  T getData() const 
  {
    // You could do locking here to prevent people from accessing while updating
    // Do an implicit copy otherwise the locking really doesn't do anything
    T value;
    CachePolicy::pullData(_name, value);
    return value;
    }

  void setData(const T &value) const
  {
    // Again doing a copy to allow for locking in the future
    CachePolicy::pushData(_name, value);
  }

  bool exist() const
  {
    return ros::param::has(_name);
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

  /*
  template <class T>
  void get(const std::string &name, const std::string &description, Parameter<T> &param) {
    param = Parameter<T>(name, description);
  }
  */
}


// This could be integrated into the ros::NodeHandle object once the API is set

class ParameterInterface {
public:
  
  template <class T, class U>
  Parameter<T,U> createParameter(const std::string &name, const T &default_value) {
    Parameter<T,U> p;
    param::get(name, p, default_value);
    return p;
  }

  /*
  template <class T>
  Parameter<T> createParameter(const std::string &name, const std::string &description) {
    Parameter<T> p;
    param::get(name, description, p);
    return p;
  }
  */  
};


  } // gsoc

} // ros

