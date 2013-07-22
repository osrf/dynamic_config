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

#include "parameter_example/Get.h"

namespace ros_parameter {

  namespace serialization {

    template <typename T>
    void serializeValue(const T& data, std::vector<uint8_t>& buffer)
    {
      buffer.resize(ros::serialization::serializationLength(data));
      ros::serialization::OStream ostream(&buffer[0], buffer.size());
      ros::serialization::serialize(ostream, data);
    }

    template < typename T >
    void deserializeValue(std::vector<uint8_t>& data, T& output)
    {
      ros::serialization::IStream istream(&data[0], data.size());
      ros::serialization::Serializer<T>::read(istream, output);  
    }

  } // serialization

  template <typename T> class Parameter;

  namespace { // anonymous

    typedef boost::variant<bool, int, double, std::string> DataTypes;
    typedef std::map<std::string, boost::any> GlobalDataMap;
    typedef std::map<std::string, boost::any> GlobalParameters;

    template <typename T>
    class GlobalParameter {
    private:
      struct all_true {
        typedef bool result_type;
        template<typename InputIterator>
        bool operator()(InputIterator first, InputIterator last) const {
          for (; first != last; ++first)
            if (!(*first)) return false;
          return true;
        }
      };

    public:
      typedef boost::signals2::connection Connection;
      typedef boost::signals2::signal<bool (const Parameter<T>&, const T&), all_true> OnChangeCallbacks;
      typedef typename OnChangeCallbacks::slot_type OnChangeCallback;
      typedef boost::signals2::signal<void (Parameter<T>&)> OnUpdateCallbacks;
      typedef typename OnUpdateCallbacks::slot_type OnUpdateCallback;

      GlobalParameter(const std::string& name) 
      : name_(name)
      , data_ptr_(new T) 
      { }

      GlobalParameter(const std::string& name, const T& data)
      : name_(name)
      , data_ptr_(new T(data))
      { 
        parameter_example::Get srv;
        srv.request.name = name;
        serialization::serializeValue(data, srv.request.data);
        srv.request.on_update_service = "the_on_update_service";
      }

      const std::string& name() const
      { return name_; }

      const T& get() const
      { return *data_ptr_; }

      const boost::shared_ptr<T>& get_ptr() const
      { return data_ptr_; }

      bool set(const boost::shared_ptr<T> data_ptr) {
        Parameter<T> param(name_);
        bool changed = on_change_callbacks_(param, *data_ptr);
        if (changed) {
          data_ptr_ = data_ptr;
          on_update_callbacks_(param);
        }
        return changed;
      }

      bool set(const T& data) {
        boost::shared_ptr<T> data_ptr(new T(data));
        return set(data_ptr);
      }

      Connection connect_on_change_callback(OnChangeCallback callback)
      { return on_change_callbacks_.connect(callback); }

      Connection connect_on_update_callback(OnUpdateCallback callback)
      { return on_update_callbacks_.connect(callback); }

    private:
      const std::string name_;
      boost::shared_ptr<T> data_ptr_;
      OnChangeCallbacks on_change_callbacks_;
      OnUpdateCallbacks on_update_callbacks_;
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

    Parameter(const std::string &name)
    : impl_(new Impl) {
      impl_->g_param_ = get_global_parameter<T>(name);
    }

    Parameter(const std::string &name, const T &default_value)
    : impl_(new Impl) {
      impl_->g_param_ = get_global_parameter<T>(name);
      impl_->g_param_->set(default_value);
    }

    Parameter(const Parameter<T> &other) 
    { impl_ = other.impl_; }

    ~Parameter() { }

    Parameter<T>& operator=(const Parameter<T>& r) 
    { impl_ = r.impl_; }

    bool operator==(const Parameter<T>& r) 
    { return impl_ == r.impl_; }

    bool operator!=(const Parameter<T>& r) 
    { return !(*this == r); }

    std::string name() const 
    { return impl_->g_param_->name(); }

    T data() const 
    { return impl_->g_param_->get(); }

    void get_data(T &data) const
    { data = impl_->g_param_->get(); }

    Ptr data_ptr() 
    { return impl_->g_param_->get_ptr(); }

    bool data_ptr(const Ptr &data_ptr) 
    { return impl_->g_param_->set(data_ptr); }

    bool data(const T &data) 
    { return impl_->g_param_->set(data); }

    void on_change(OnChangeCallbackType callback) 
    { impl_->on_change_connection = impl_->g_param_->connect_on_change_callback(callback); }

    void on_update(OnUpdateCallbackType callback) 
    { impl_->on_update_connection = impl_->g_param_->connect_on_update_callback(callback); }

  private:
    struct Impl {
      ~Impl() {
        on_change_connection.disconnect();
        on_update_connection.disconnect();
      }
      boost::shared_ptr<GlobalParameter<T> > g_param_;
      typename GlobalParameter<T>::Connection on_change_connection;
      typename GlobalParameter<T>::Connection on_update_connection;
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