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
#include <boost/signals2.hpp>

#include "ros/this_node.h"

namespace ros_parameter {

  template <typename T> class Parameter;

  namespace { // anonymous

    typedef boost::variant<bool, int, double, std::string> DataTypes;
    typedef std::map<std::string, boost::any> GlobalDataMap;
    typedef std::map<std::string, boost::any> GlobalParameters;

    struct all_true {
      typedef bool result_type;
      template<typename InputIterator>
      bool operator()(InputIterator first, InputIterator last) const
      {
        for (; first != last; ++first) {
          if (!(*first)) return false;
        }
        return true;
      }
    };

    template <typename T>
    struct GlobalParameter {
      GlobalParameter(const std::string& name) 
      : name_(name)
      , data_ptr_(new T) 
      { }

      const std::string name_;
      boost::shared_ptr<T> data_ptr_;
      boost::signals2::signal<bool (const Parameter<T>&, const T&), all_true> on_change_callbacks_;
      boost::signals2::signal<void (Parameter<T>&)> on_update_callbacks_;
    };

    GlobalParameters g_parameters;

    template <typename T>
    inline boost::shared_ptr<GlobalParameter<T> > create_global_parameter(const std::string& name) {
      boost::shared_ptr<GlobalParameter<T> > ptr(new GlobalParameter<T>(name));
      g_parameters[name] = ptr;
      return ptr;
    }

    template <typename T>
    inline boost::shared_ptr<GlobalParameter<T> > cast_parameter_from_any(const boost::any& any_param) {
      return boost::any_cast<boost::shared_ptr<GlobalParameter<T> > >(any_param);
    }

    template <typename T>
    boost::shared_ptr<GlobalParameter<T> > get_global_parameter(const std::string& name) {
      GlobalParameters::iterator it = g_parameters.find(name);
      GlobalParameters::iterator end = g_parameters.end();
      return ( it == end ? create_global_parameter<T>(name) : cast_parameter_from_any<T>(it->second) );
    }

  } // anonymous

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
      impl_->g_param_ = get_global_parameter<T>(name);
    }

    Parameter(const std::string &name, const T &default_value)
    : impl_(new Impl) {
      impl_->g_param_ = get_global_parameter<T>(name);
      get_data_ptr_helper().reset(new T(default_value));
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
      return *get_data_ptr_helper();
    }

    void get_data(T &data) {
      data = *get_data_ptr_helper();
    }

    Ptr data_ptr() {
      return get_data_ptr_helper();
    }

    void data_ptr(const Ptr &data_ptr) {
      impl_->g_param_->data_ptr_ = data_ptr;      
    }

    bool data(const T &data) {
      bool change = change_data(data);      
      if (change)
        update_data(data);
      return change;
    }

    std::string name() const {
      return get_name_helper();
    }

    void on_change(OnChangeCallbackType callback) {
      impl_->on_change_scoped_connection =
        get_on_change_callbacks_helper().connect(callback);
    }

    void on_update(OnUpdateCallbackType callback) {
      impl_->on_update_scoped_connection = 
        get_on_update_callbacks_helper().connect(callback);
    }

  private:

    bool change_data(const T& data) {
      return get_on_change_callbacks_helper()(*this, data);
    }

    void update_data(const T& data) {
      get_data_ptr_helper().reset(new T(data));
      get_on_update_callbacks_helper()(*this);
    }

    inline boost::shared_ptr<T>& get_data_ptr_helper() {
      return impl_->g_param_->data_ptr_;
    }

    inline const boost::shared_ptr<T>& get_data_ptr_helper() const {
      return impl_->g_param_->data_ptr_;
    }

    inline const std::string& get_name_helper() const {
      return impl_->g_param_->name_;
    }

    inline boost::signals2::signal<bool (const Parameter<T>&, const T&), all_true>& get_on_change_callbacks_helper() {
      return impl_->g_param_->on_change_callbacks_;
    }

    inline boost::signals2::signal<void (Parameter<T>&)>& get_on_update_callbacks_helper() {
      return impl_->g_param_->on_update_callbacks_;
    }

    struct Impl {
      boost::shared_ptr<GlobalParameter<T> > g_param_;
 
      OnChangeCallbackType on_change_callback_;
      OnUpdateCallbackType on_update_callback_;

      boost::signals2::scoped_connection on_change_scoped_connection;
      boost::signals2::scoped_connection on_update_scoped_connection;
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
    typedef std::map<std::string, ParameterType> Parameters;

    ParameterGroup()
    : impl_(new Impl) {
      impl_->namespace_ = ros::this_node::getNamespace();
    }

    ParameterGroup(const std::string& nm)
    : impl_(new Impl) {
      impl_->namespace_ = nm;
    }

    ~ParameterGroup() { }

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

    const std::string& get_namespace() const {
      return impl_->namespace_;
    }

    template <typename T>
    Parameter<T> create_parameter(const std::string& name, const T& default_value) {
      std::string ns = impl_->namespace_;
      ns += ( *ns.rbegin() == '/' ? "" : "/" );
      ns += name;

      Parameter<T> param(ns, default_value);
      impl_->parameters_[name] = param;

      param.on_change(boost::bind(&ParameterGroup::parameter_on_change<T>, this, _1, _2));
      param.on_update(boost::bind(&ParameterGroup::parameter_on_update<T>, this, _1));

      return param;
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
      std::string namespace_;
      OnChangeCallbackType on_change_callback_;
      OnUpdateCallbackType on_update_callback_;
      Parameters parameters_;
    };
    typedef boost::shared_ptr<Impl> ImplPtr;
    ImplPtr impl_;
  };

} // namespace ros_parameter