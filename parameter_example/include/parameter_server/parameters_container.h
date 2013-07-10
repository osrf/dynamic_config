#ifndef PARAMETERS_CONTAINER_H
#define PARAMETERS_CONTAINER_H

#include <map>
#include <stdexcept>

typedef typename std::runtime_error NonExistentParameter;

template <class T>
class ParametersContainer
{
  typedef typename std::map<std::string, T> Parameters;

public:
  typedef std::set<std::string> Keys;

  void set(const std::string& name, const T& value)
  {
    parameters_[name] = value;
  }

  void get(const std::string& name, T& value)
  {
    typename Parameters::iterator it = parameters_.find(name);
    if (it == parameters_.end())
      throw NonExistentParameter("Parameter " + name + " doesn't exist");
    value = it->second;
  }

  void remove(const std::string& name)
  {
    parameters_.erase(name);
  }

  bool has(const std::string& name)
  {
    return parameters_.find(name) != parameters_.end();
  }

  void keys(Keys& theKeys) {
    typename Parameters::iterator it = parameters_.begin();
    typename Parameters::iterator end = parameters_.end();
    for (; it != end; ++it) {
      theKeys.insert(it->first);
    }
  }

private:
  Parameters parameters_;
};

#endif // PARAMETERS_CONTAINER_H
