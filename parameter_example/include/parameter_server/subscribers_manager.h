#ifndef SUBSCRIBERS_MANAGER_H
#define SUBSCRIBERS_MANAGER_H

#include <map>
#include <list>
#include <algorithm>
#include <boost/bind.hpp>

namespace parameter_server {

template <class Notifier>
class SubscribersManager
{
  typedef std::list<Notifier> ParamNotifiers;
  typedef std::map<std::string, ParamNotifiers> Notifiers;

  public:
    SubscribersManager() {};
    ~SubscribersManager() {};

    void add(const std::string& param, const Notifier& notifier)
    {
      notifiers_[param].push_back(notifier);
    }

    int size(const std::string& param)
    { 
      return notifiers_[param].size(); 
    }

    void remove(const std::string& param, const Notifier& notifier)
    {
      notifiers_[param].remove(notifier);
    }

    void notify(const std::string& param, int paramEvent)
    {
      std::for_each(notifiers_[param].begin(),
                    notifiers_[param].end(),
                    boost::bind(&Notifier::notify, _1, paramEvent));
    }

  private:
    Notifiers notifiers_;
};

} // parameter_server

#endif
