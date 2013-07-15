#ifndef PARAMETER_H
#define PARAMETER_H

#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>
#include <set>

namespace parameter_server {

template <class T>
class Parameter
{
public:

  struct CallbackResponse
  {
    int event;
    T value;
  };

private:

  typedef boost::signals2::signal<void (const CallbackResponse&)> Subscribers;
  typedef typename Subscribers::slot_type SubscriberCallback;  

public:

  enum Event { CREATED, UPDATED, DELETED };

  Parameter()
  : impl_(new Impl)
  { impl_->setted = false; }

  ~Parameter()
  { }

  Parameter(const Parameter& other)
  : impl_(other.impl_) { }

  Parameter& operator=(const Parameter& rhs)
  { impl_ = rhs.impl_; }

  bool operator==(const Parameter& rhs) const
  { return impl_ == rhs.impl_; }

  void setValue(const T& value)
  { 
    impl_->value = value;
    impl_->setted = true;
  }

  void getValue(T& value)
  { value = impl_->value; }

  bool hasValue()
  { return impl_->setted; }

  void deleteValue()
  { impl_->setted = false; }

  template <class U>
  void subscribe(bool(U::*cb)(const CallbackResponse&), boost::shared_ptr<U>& ptr)
  {
    namespace bs2 = boost::signals2;
    impl_->sig.connect(SubscriberCallback(cb, ptr.get(), _1).track(ptr));
  }

  void notify(const CallbackResponse& res)
  {
    impl_->sig(res);
  }

private:

  struct Impl
  {
    T value;
    bool setted;
    Subscribers sig;
  };
  typedef boost::shared_ptr<Impl> ImplPtr;

  ImplPtr impl_;
};

}

#endif