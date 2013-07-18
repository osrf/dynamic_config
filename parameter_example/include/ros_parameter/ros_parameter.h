/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation, Inc. nor
 *     the names of its contributors may be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <map>
#include <string>
#include <vector>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/variant.hpp>
#include <boost/any.hpp>

namespace ros_parameter {

  namespace { // anonymous

    typedef boost::variant<bool, int, double, std::string> DataTypes;
    typedef std::map<std::string, boost::any> GlobalDataMap;
    GlobalDataMap g_dataMap;

    template <typename T>
    boost::shared_ptr<T> get_global_data_ptr(const std::string& name) {
      boost::any thePtr = g_dataMap[name];
      return boost::any_cast<boost::shared_ptr<T> >(thePtr);
    }

    bool has_global_data(const std::string& name) {
      return g_dataMap.find(name) != g_dataMap.end();
    }

    template <typename T>
    T& get_global_data(const std::string& name) {
      if ( !has_global_data(name) ) {
        T default_value;
        set_global_data(name, default_value);
      }
      return *get_global_data_ptr<T>(name);
    }

    template <typename T>
    void set_global_data_ptr(const std::string& name, const boost::shared_ptr<T>& ptr) {
      g_dataMap[name] = ptr;
    }

    template <typename T>
    void set_global_data(const std::string& name, const T& data) {
      boost::shared_ptr<T> ptr(new T(data));
      set_global_data_ptr(name, ptr);
    }

  } // anonyous

  template <class T>
  class Parameter
  {
  public:
    typedef boost::shared_ptr<T> Ptr;
    typedef boost::function<bool (const Parameter<T>&, const T&)> OnChangeCallbackType;
    typedef boost::function<void (Parameter<T>&)> OnUpdateCallbackType;

    Parameter() : impl_(new Impl) {}
    ~Parameter() {}

    Parameter(const std::string &name)
    : impl_(new Impl) {
      impl_->name_ = name;
    }

    Parameter(const std::string &name, const T &default_value)
    : impl_(new Impl) {
      impl_->name_ = name;
      set_global_data(name, default_value);
    }

    Parameter(const Parameter<T> &other) {
      impl_ = other.impl_;
    }

    Parameter<T>& operator=(const Parameter<T>& r) {
      impl_ = r.impl_;
    }

    bool operator==(const Parameter<T>& r) {
      return impl_ == r.impl_;
    }

    bool operator!=(const Parameter<T>& r) {
      return !(*this == r);
    }

    T data() const {
      return get_global_data<T>(impl_->name_);
    }

    void get_data(T &data) {
      data = get_global_data<T>(impl_->name_);
    }

    Ptr data_ptr() {
      return get_global_data<T>(impl_->name_);
    }

    void data_ptr(const Ptr &data_ptr) {
      set_global_data_ptr(data_ptr);
    }

    bool data(const T &data) {
      bool change = change_data(data);
      if (change)
        update_data(data);
      return change;
    }

    std::string name() const {
      return impl_->name_;
    }

    void name(const std::string &name) {
      name = impl_.name_;
    }

    void on_change(OnChangeCallbackType callback) {
      impl_->on_change_callback_ = callback;
    }

    void on_update(OnUpdateCallbackType callback) {
      impl_->on_update_callback_ = callback;
    }

  private:

    bool change_data(const T& data) {
      if (impl_->on_change_callback_) {
        return impl_->on_change_callback_(*this, data);
      }
      return true;
    }

    void update_data(const T& data) {
      set_global_data(impl_->name_, data);
      if (impl_->on_update_callback_)
        impl_->on_update_callback_(*this);      
    }

    struct Impl {
      std::string name_;
      OnChangeCallbackType on_change_callback_;
      OnUpdateCallbackType on_update_callback_;
    };
    typedef boost::shared_ptr<Impl> ImplPtr;
    ImplPtr impl_;    
  };

  class ParameterGroup
  {
  public:
    typedef DataTypes DataType;
    typedef boost::function<void(const ParameterGroup&, std::map<std::string,DataType>&, 
                            std::map<std::string, bool>&)> OnChangeCallbackType;
    typedef boost::function<void(ParameterGroup&, std::map<std::string, bool>&)> OnUpdateCallbackType;
    typedef boost::variant<Parameter<bool>, Parameter<int>, Parameter<double>, Parameter<std::string> > ParameterType;

    ParameterGroup() : impl_(new Impl) {}

    ~ParameterGroup() {}

    ParameterGroup(const ParameterGroup &other) {
      impl_ = other.impl_;
    }

    ParameterGroup& operator=(const ParameterGroup& r) {
      impl_ = r.impl_;
    }

    bool operator==(const ParameterGroup& r) {
      return impl_ == r.impl_;
    }

    bool operator!=(const ParameterGroup& r) {
      return !(*this == r);
    }

    template <typename T>
    void add_parameter(const Parameter<T> param) {
      std::string name = param.name();
      impl_->parameters_[name] = param;

      get<T>(name).on_change(boost::bind(&ParameterGroup::parameter_on_change<T>, this, _1, _2));
      get<T>(name).on_update(boost::bind(&ParameterGroup::parameter_on_update<T>, this, _1));
    }

    void on_change(OnChangeCallbackType callback) {
      impl_->on_change_callback_ = callback;
    }

    void on_update(OnUpdateCallbackType callback) {
      impl_->on_update_callback_ = callback;
    }

    template <typename T>
    Parameter<T> get(const std::string &name) const {
      return boost::get<Parameter<T> >(impl_->parameters_[name]);
    }

    template <typename T>
    void get_data(const std::string &name, T &data) const {
      Parameter<T> param = this->get<T>(name);
      param.get_data(data);
    }

  private:

    template <class T>
    bool parameter_on_change(const Parameter<T>& param, const T& data) {
      if (!impl_->on_change_callback_)
        return true;

      std::string name = param.name();
      std::map<std::string, bool> changed;
      fillMapWithFalseExceptOneParameter(name, changed);

      std::map<std::string, DataType> newData;
      newData[name] = data;

      impl_->on_change_callback_(*this, newData, changed);

      return changed[name];
    }

    template <typename T>
    void parameter_on_update(Parameter<T>& param) {
      if (!impl_->on_update_callback_)
        return;

      std::string name = param.name();
      std::map<std::string, bool> changed;
      fillMapWithFalseExceptOneParameter(name, changed);

      impl_->on_update_callback_(*this, changed);
    }

    void fillMapWithFalseExceptOneParameter(const std::string& name, std::map<std::string, bool>& changed) {
      std::map<std::string, ParameterType>::iterator it = impl_->parameters_.begin();
      std::map<std::string, ParameterType>::iterator end = impl_->parameters_.end();
      for (; it != end; ++it) {
        changed[it->first] = false;
      }
      changed[name] = true;
    }

    struct Impl {
      OnChangeCallbackType on_change_callback_;
      OnUpdateCallbackType on_update_callback_;
      std::map<std::string, ParameterType> parameters_;
    };
    typedef boost::shared_ptr<Impl> ImplPtr;
    ImplPtr impl_;
  };

} // namespace ros_parameter
