#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

namespace ros {

  namespace gsoc {

template <class T>
class Parameter {
public:
  Parameter(const std::string &name, const std::string &description, const T &default_value)
  : _data(new T)
  {
    ros::param::param<T>(name, *(this->_data), default_value);
  }
  Parameter(const std::string &name, const std::string &description)
  : _data(new T)
  {
    ros::param::get(name, *(this->_data));
  }
  Parameter() : _data(new T) {}
  // Copy Constructors
  Parameter(const Parameter &other) : _data(other._data) {}
  Parameter(const T &value) {
    this->set_data(value);
  }
  ~Parameter() {}

  T get_data() const {
    // You could do locking here to prevent people from accessing while updating
    // Do an implicit copy otherwise the locking really doesn't do anything
    return *(this->_data);
  }

  void set_data(const T &value) {
    // Again doing a copy to allow for locking in the future
    this->_data = boost::shared_ptr<T>(new T(value));
  }

private:
  boost::shared_ptr<T> _data;
};

namespace param {
  template <class T>
  void get(const std::string &name, const std::string &description, Parameter<T> &param, const T &default_value)
  {
    param = Parameter<T>(name, description, default_value);
  }

  template <class T>
  void get(const std::string &name, const std::string &description, Parameter<T> &param) {
    param = Parameter<T>(name, description);
  }
}

// This could be integrated into the ros::NodeHandle object once the API is set
class ParameterInterface {
public:
  template <class T>
  Parameter<T> createParameter(const std::string &name, const std::string &description, const T &default_value) {
    Parameter<T> p;
    param::get(name, description, p, default_value);
    return p;
  }

  template <class T>
  Parameter<T> createParameter(const std::string &name, const std::string &description) {
    Parameter<T> p;
    param::get(name, description, p);
    return p;
  }
};

  } // gsoc

} // ros

