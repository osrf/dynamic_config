#ifndef PARAMETERS_SERVER_H
#define PARAMETERS_SERVER_H

#include "parameter_server/parameters_container.h"
#include "parameter_server/subscribers_manager.h"
#include "parameter_server/commands_queue.h"
#include "parameter_server/command.h"

namespace parameter_server {

template <class T, class Notifier>
class ParametersServer
{
 public:
  bool set(const std::string& name, const T& value)
  {    
    const int event = 0;
    commands_.add(command::make_command(&SubscribersManager<Notifier>::notify,
                                        &subscribers_, name, event));
    values_.set(name, value);
  }

  void get(const std::string& name, T& value)
  {
    try {
      values_.get(name, value);
    } catch (std::exception& ex) {
      // Do nothing
    }
  }

  void subscribe(const std::string& name, const std::string& subscriber_id) {
    commands_.add(command::make_command_const(&SubscribersManager<Notifier>::add,
                                        &subscribers_, name, subscriber_id));
    subscribers_.add(name, subscriber_id);
  }

  void processSubscribersEvents()
  {
    subscribers_.executeAll();
  }

 private:
  ParametersContainer<T> values_;
  SubscribersManager<Notifier> subscribers_;
  command::CommandsQueue<command::Command> commands_;
};

} // parameter_server

#endif
