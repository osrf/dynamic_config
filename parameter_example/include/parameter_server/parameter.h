#ifndef PARAMETER_H
#define PARAMETER_H

#include <boost/shared_ptr.hpp>

namespace parameter_server {

template <class T>
class Parameter
{
public:
  Parameter()
  : impl_(new Impl)
  { }

  ~Parameter()
  { }

  Parameter(const Parameter& other)
  : impl_(other.impl_) { }

  Parameter& operator=(const Parameter& rhs)
  { impl_ = rhs.impl_; }

  bool operator==(const Parameter& rhs) const
  {
    return impl_ == rhs.impl_;
  }

private:

  struct Impl
  {
    T value;
  };
  typedef boost::shared_ptr<Impl> ImplPtr;

  ImplPtr impl_;
};

}

#endif