#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

namespace ros {

  namespace gsoc {

template <class T>
class Parameter {
public:
  Parameter(ros::NodeHandle &nh, const std::string &name, const std::string &description, const T &default_value)
  : _data(new T)
  {
    ros::param::param<T>(name, *(this->_data), default_value);
  }
  Parameter(ros::NodeHandle &nh, const std::string &name, const std::string &description)
  : _data(new T)
  {
    ros::param::get(name, *(this->_data));
  }
  ~Parameter();

  T get_data() const {
    // You could do locking here to prevent people from accessing while updating
    // Do an implicit copy otherwise the locking really doesn't do anything
    return *(this->_data);
  }

private:
  boost::shared_ptr<T> _data;
};

class NodeHandle: public ros::NodeHandle {
public:
  template <class T>
  Parameter<T> createParameter(const std::string &name, const std::string &description, const T &default_value) {
    return Parameter<T>(*(this), name, description, default_value);
  }

  template <class T>
  Parameter<T> createParameter(const std::string &name, const std::string &description) {
    return Parameter<T>(*(this), name, description);
  }
};

  } // gsoc

} // ros

