#ifndef COMMAND_H
#define COMMAND_H

#include <boost/function.hpp>
#include <boost/bind.hpp>

namespace parameter_server {

  namespace command {

class Command 
{ 
  typedef boost::function<void ()> Function;

 public: 
  Command(const Function& f)
  : f_(f) { }
  
  Command(const Command& other)
  : f_(other.f_) { }

  void operator()()
  { f_(); }

 private:
  boost::function<void ()> f_;
};

template <class T>
Command make_command(void(T::*func)(), T *obj)
{
  return Command(boost::bind(func, obj));
}


template <class T, class Arg1>
Command make_command(void(T::*func)(const Arg1&), 
                     T *obj, const Arg1& arg1)
{
  return Command(boost::bind(func, obj, arg1));
}

template <class T, class Arg1, class Arg2>
Command make_command(void(T::*func)(const Arg1&, const Arg2&), 
                     T *obj, const Arg1& arg1, const Arg2& arg2)
{
  return Command(boost::bind(func, obj, arg1, arg2));
}

/*
template <class T, class Arg1>
class Command1Arg : public Command
{
 public:
  Command1Arg(void(T::*func)(const Arg1&, const Arg2&), T *obj
                  const Arg1& arg1)
    : Command(boost::bind(func, obj, arg1))
    { }
};

template <class T, class Arg1>
class Command1Arg : public Command
{
 public:
  Command1Arg(void(T::*func)(const Arg1&, const Arg2&), T *obj
                  const Arg1& arg1)
    : Command(boost::bind(func, obj, arg1))
    { }
};
*/

  } // command

} // parameter_server

#endif
