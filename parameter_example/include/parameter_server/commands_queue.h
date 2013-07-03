#ifndef COMMANDS_QUEUE_H
#define COMMANDS_QUEUE_H

#include <queue>
#include <algorithm>
#include <boost/thread/mutex.hpp>

namespace parameter_server {

  namespace command {

// This class is thread safe. All commands are stored in the
// *current* queue. When user call executeAll() all commands
// in the *current* queue are executed. If a command is added
// while executing the commands, the new command is stored in
// the new *current* queue and they will be executed in the 
// next executeAll() call.

// If executeAll() is called while the the queue is executing
// the method will block until the other queue finish executing
// and executes the current commands.
template <class T>
class CommandsQueue
{
 public:
  CommandsQueue() 
  : current_(&queue1_) 
  , executable_(&queue2_)  
  {}

  ~CommandsQueue() {}

  void add(const T& command)
  {
    boost::mutex::scoped_lock currLock(currMutex_);
    current_->push(command);
  }

  void executeAll()
  {
    boost::mutex::scoped_lock execLock(execMutex_);
    swapQueuePointers();
    executeExecutableQueue();
  }

  void executing()
  {
    boost::try_mutex::scoped_try_lock execLock(execMutex_);
    return !execLock;
  }

  int size()
  {
    boost::mutex::scoped_lock currLock(currMutex_); 
    return current_->size();
  }

 private:
  void swapQueuePointers()
  {
    boost::mutex::scoped_lock currLock(currMutex_);
    std::swap(current_, executable_);    
  }

  void executeExecutableQueue()
  {
    while (!executable_->empty()) {
      executable_->front()();
      executable_->pop();
    }
  }

  std::queue<T> queue1_;
  std::queue<T> queue2_;
  std::queue<T>* current_;
  std::queue<T>* executable_;
  boost::mutex currMutex_;
  boost::mutex execMutex_;
};

  } // command

} // parameter_server

#endif
