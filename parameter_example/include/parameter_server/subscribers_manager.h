#ifndef SUBSCRIBERS_MANAGER_H
#define SUBSCRIBERS_MANAGER_H

#include <map>
#include <set>
#include <algorithm>
#include <boost/bind.hpp>

namespace parameter_server {

template <class Notifier>
class SubscribersManager : public Notifier
{
  struct Subscriber
  {
    explicit Subscriber(const std::string& id)
    : id_(id), tries_(0) { }
    bool operator==(const Subscriber &other) const
    { return id_ == other.id_; }
    bool operator==(const std::string &other) const
    { return id_ == other; }
    bool operator<(const Subscriber& rhs) const
    { return id_ < rhs.id_; }
    void reset() const
    { tries_ = 0; }
    void fail() const
    { ++tries_; }
    std::string id_;
    mutable int tries_;
  };
  typedef std::set<Subscriber> Subscribers;
  typedef std::map<std::string, Subscribers> SubscribersMap;

  public:
    SubscribersManager()
    : maxTries_(3)
    { }

    ~SubscribersManager()
    { }

    void add(const std::string& param, const std::string& subscriber)
    {
      subscribers_[param].insert(Subscriber(subscriber));
    }

    void remove(const std::string& param, const std::string& subscriber)
    {
      subscribers_[param].erase(Subscriber(subscriber));
    }

    void tries(int number)
    {
      maxTries_ = number;
    }

    void notify(const std::string& param, int event)
    {
      typename Subscribers::iterator subscriber = subscribers_[param].begin();
      typename Subscribers::iterator end = subscribers_[param].end();
      for (; subscriber != end; ++subscriber)
        if (Notifier::call(param, subscriber->id_, event)) {
          subscriber->reset();
        } else {
          subscriber->fail();
          if (subscriber->tries_ >= maxTries_-1)
            remove(param, subscriber->id_);
        }
    }

  private:
    SubscribersMap subscribers_;
    int maxTries_;

};

} // parameter_server

#endif
