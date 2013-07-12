#ifndef PARAMETER_H
#define PARAMETER_H

#include <boost/shared_ptr.hpp>
#include <set>

namespace parameter_server {

template <class T, class Subscriber>
class Parameter
{
  typedef std::set<Subscriber> Subscribers;

public:
  typedef typename Subscribers::iterator iterator;

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

  void addSubscriber(const Subscriber& subscriber)
  { impl_->subscribers.insert(subscriber); }

  void removeSubscriber(const Subscriber& subscriber)
  { impl_->subscribers.erase(subscriber); }

  iterator subscribersBegin()
  { return impl_->subscribers.begin(); }

  iterator subscribersEnd()
  { return impl_->subscribers.end(); }

  int subscribersSize()
  { return impl_->subscribers.size(); }

  bool subscribersEmpty()
  { return impl_->subscribers.empty(); }

private:

  struct Impl
  {
    T value;
    bool setted;
    Subscribers subscribers;
  };
  typedef boost::shared_ptr<Impl> ImplPtr;

  ImplPtr impl_;
};

}

#endif