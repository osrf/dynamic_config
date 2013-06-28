#include <boost/shared_ptr.hpp>
#include <boost/signal.hpp>
#include <map>
#include <ros/ros.h>

#include "parameter_example/SetParam.h"
#include "parameter_example/GetParam.h"
#include "parameter_example/HasParam.h"
#include "parameter_example/SearchParam.h"
#include "parameter_example/DeleteParam.h"
#include "parameter_example/SubscribeParam.h"
#include "parameter_example/UnsubscribeParam.h"
#include "parameter_example/ParamEvent.h"

namespace ros {

  namespace gsoc {
    
    namespace serialization {

      template <typename T>
      void serializeValue(const T& data, std::vector<uint8_t>& buffer)
      {
        buffer.resize(ros::serialization::serializationLength(data));
        ros::serialization::OStream ostream(&buffer[0], buffer.size());
        ros::serialization::serialize(ostream, data);
      }

      template < typename T >
      void deserializeValue(std::vector<uint8_t>& data, T& output)
      {
        ros::serialization::IStream istream(&data[0], data.size());
        ros::serialization::Serializer<T>::read(istream, output);  
      }

    } // serialization

// To get and set the value of the parameter it uses
// the policy specified.
template <typename T>
class Parameter
{
public:
 Parameter(const std::string& name, boost::function<void (int, T)>& param_func)
  : impl_(new Impl)
  {
    impl_->name_ = name;
    impl_->serviceServerRunning_ = false;
    impl_->sig_.connect(param_func);
    subscribe();
  }

  Parameter(const std::string &name, const T &default_value)
  : impl_(new Impl)
  { 
    impl_->name_ = name;
  }

  Parameter(const std::string &name)
  : impl_(new Impl)
  { 
    impl_->name_ = name;
  }

  Parameter() {}

  // Copy Constructors
  Parameter(const Parameter &other) 
  : impl_(other.impl_) 
  {}

  ~Parameter() { }

  const std::string& getName()
  {
    return impl_->name_;
  }

  T getData() const
  {
    // You could do locking here to prevent people from accessing while updating
    // Do an implicit copy otherwise the locking really doesn't do anything
    parameter_example::GetParam srv;
    srv.request.name = impl_->name_;
    std::string srv_name("/parameter_server/get_param");
    T value;
    if (ros::service::call(srv_name, srv)) {
      serialization::deserializeValue(srv.response.value, value);
    } else {
      ROS_ERROR_STREAM("Unable to call '" << srv_name << "' service");
      // throw ParamNotExistException
    }    
    return value;
  }

  void setData(const T &value)
  {
    // Again doing a copy to allow for locking in the future
    parameter_example::SetParam srv;
    srv.request.name = impl_->name_;
    serialization::serializeValue(value, srv.request.value);
    std::string srv_name("/parameter_server/set_param");
    if (ros::service::call(srv_name, srv)) {
      // Do nothing  
    } else {
      ROS_ERROR_STREAM("Unable to call '" << srv_name << "' service");
    }
  }

  bool exist()
  {
    parameter_example::HasParam srv;
    srv.request.name = impl_->name_;
    std::string srv_name("/parameter_server/has_param");
    if (ros::service::call(srv_name, srv)) {
      return srv.response.hasParam;
    } else {
      ROS_ERROR_STREAM("Unable to call '" << srv_name << "' service");
    }
    return false;
  }

  void del()
  {
  }

  // Should not be visible
  bool notifyCallbacks(parameter_example::ParamEvent::Request  &req,
                       parameter_example::ParamEvent::Response &res)
  {
    T value;
    serialization::deserializeValue(req.value, value);
    impl_->sig_(req.event, value);
  }

private:

  void subscribe()
  {
    // create ServiceServer
    if (!impl_->serviceServerRunning_) {
      impl_->serviceServer_ = ros::NodeHandle("~")
        .advertiseService("param_event", &Parameter<T>::notifyCallbacks, this);
      impl_->serviceServerRunning_ = false;
    }

    // subscribe
    parameter_example::SubscribeParam srv;
    srv.request.name = impl_->name_;
    srv.request.service = impl_->serviceServer_.getService();
    std::string srv_name("/parameter_server/subscribe_param");
    if (ros::service::call(srv_name, srv)) {
      // Do nothing  
    } else {
      ROS_ERROR_STREAM("Unable to call '" << srv_name  << "' service");
    }
  }

  struct Impl {
    std::string name_;
    bool serviceServerRunning_;
    ros::ServiceServer serviceServer_;
    boost::signal<void (int, T)> sig_;
  };
  typedef boost::shared_ptr<Impl> ImplPtr;

  ImplPtr impl_;
};


namespace param {

  template <class T>
  void get(const std::string &name, Parameter<T> &param, const T &default_value)
  {
    param = Parameter<T>(name, default_value);
  }

  template <class T>
  void get(const std::string &name, Parameter<T> &param) {
    param = Parameter<T>(name);
  }
  
}


// This could be integrated into the ros::NodeHandle object once the API is set

class ParameterInterface {
public:

  template <class T>
  Parameter<T> createParameter(const std::string& name,
                               boost::function<void (int, T)>& param_func)
  {
    Parameter<T> p(name, param_func);
    return p;
  }
  
  template <class T>
  Parameter<T> createParameter(const std::string &name, 
				 const T &default_value) 
  {
    Parameter<T> p;
    param::get(name, p, default_value);
    return p;
  }

  template <class T>
  Parameter<T> createParameter(const std::string &name) 
  {
    Parameter<T> p;
    param::get(name, p);
    return p;
  }

};

  } // gsoc

} // ros

