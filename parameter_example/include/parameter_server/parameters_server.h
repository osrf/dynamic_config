#ifndef PARAMETERS_SERVER_H
#define PARAMETERS_SERVER_H

#include <map>
#include "parameter_server/parameter.h"
#include <boost/thread/mutex.hpp>


namespace parameter_server {

template <class Value>
class ParametersServer
{

  typedef Parameter<Value> MyParameter;

public:

  typedef typename MyParameter::CallbackResponse Response;

  enum {CREATED, UPDATED, REMOVED};

  void set(const std::string& name, const Value& value)
  {
    Response res;
    res.event = has(name) ? UPDATED : CREATED;
    res.value = value;
    parameters_[name].notify(res);
    parameters_[name].setValue(value);
  }

  bool get(const std::string& name, Value& value)
  { 
    typename Parameters::iterator it = parameters_.find(name);
    bool exist = it != parameters_.end();
    if (exist) {
      it->second.getValue(value);
    }
    return exist;
  }

  bool has(const std::string& name)
  {
    typename Parameters::iterator it = parameters_.find(name);
    return it != parameters_.end() && 
           it->second.hasValue();
  }

  void remove(const std::string& name)
  {
    if (has(name)) { 
      Response res;
      res.event = REMOVED;
      parameters_[name].getValue(res.value);
      parameters_[name].notify(res);
    }
    parameters_.erase(name);
  }

  template <class U>
  void subscribe(const std::string& name, bool(U::*cb)(const Response&), boost::shared_ptr<U>& ptr)
  {
    boost::mutex::scoped_lock currLock(mutex_);
    parameters_[name].subscribe(cb, ptr);
  }

private:
  typedef std::map<std::string, MyParameter> Parameters;

  Parameters parameters_;
  boost::mutex mutex_;
};

} // parameter_server

#endif