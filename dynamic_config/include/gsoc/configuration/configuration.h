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
#include <iterator>
#include <algorithm>
#include <boost/variant.hpp>
#include <boost/bind.hpp>

namespace gsoc {

  namespace configuration {

    namespace {

      typedef boost::variant<std::string, int> Parameter;
      typedef std::map<std::string, Parameter> Parameters;

      template <typename Result, typename Visitor>
      struct  UnaryOperation : boost::static_visitor<Result> {

        UnaryOperation(const std::string& name, const Visitor& visitor)
        : name_(name)
        , visitor_(visitor)
        { }

        template <typename T> 
        Result operator()(T& t) const {
          return visitor_(std::make_pair(name_, t));
        }

        const std::string& name_;
        const Visitor& visitor_;
      };

      template <typename Result, typename Visitor>
      struct Operation {

        Operation(const Visitor& visitor)
        :visitor_(visitor)
        { }

        Result operator()(std::pair<const std::string, Parameter>& pair) const {
          UnaryOperation<Result,Visitor> op(pair.first, visitor_);
          return boost::apply_visitor(op, pair.second);
        }

        const Visitor& visitor_;
      };

      template <typename Result, typename Visitor>
      Operation<Result,Visitor> make_operation(const Visitor& visitor) {
        return Operation<Result,Visitor>(visitor);
      }

      bool sameName(std::pair<const std::string, Parameter> p1,
                    std::pair<const std::string, Parameter> p2) {
        return p1.first == p2.first;
      }

      bool sameType(std::pair<const std::string, Parameter> p1,
                    std::pair<const std::string, Parameter> p2) {
        return p1.second.type() == p2.second.type();
      }

      bool sameValue(std::pair<const std::string, Parameter> p1,
                    std::pair<const std::string, Parameter> p2) {
        return p1.second == p2.second;
      }

    } // anonymous

    class Configuration {
    public:
      Configuration() { }

      template <typename T>
      const T& get(const std::string& name) const {
        return boost::get<T>(params_[name]);
      }

      template <typename T>
      void put(const std::string& name, const T& t) {
        params_[name] = t;
      }

      template <typename Visitor>
      void apply(const std::string& name, Visitor visitor) {
        Parameters::iterator it = params_.find(name);
        if (it != params_.end())
          std::for_each(it, ++it, make_operation<void>(visitor));
        else
          std::cerr << "Param " << name << " does not exist" << std::endl;
      }

      template <typename ResultType, typename Visitor>
      ResultType apply(const std::string& name, Visitor visitor) {
        std::vector<ResultType> result(1);
        Parameters::iterator it = params_.find(name);
        if (it != params_.end())
          std::transform(it, ++it, result.begin(), make_operation<ResultType>(visitor));
        else
          std::cerr << "Param " << name << " does not exist" << std::endl;
        return result[0];
      }

      template <typename Visitor>
      void applyAll(Visitor visitor) {
        std::for_each(params_.begin(), params_.end(), make_operation<void>(visitor));
      }

      template <typename ResultType, typename Visitor, typename OutputIterator>
      void applyAll(Visitor visitor, OutputIterator result) {
        std::transform(params_.begin(), params_.end(), result, make_operation<ResultType>(visitor));
      }

      bool equivalent(const Configuration& conf) {
        return params_.size() == conf.params_.size() &&
               std::equal(params_.begin(), params_.end(), conf.params_.begin(), sameName) &&
               std::equal(params_.begin(), params_.end(), conf.params_.begin(), sameType);
      }

      

      int size() const {
        return params_.size();
      }

      bool operator==(const Configuration& rhs) {
        return equivalent(rhs) &&
               std::equal(params_.begin(), params_.end(), rhs.params_.begin(), sameValue);
      }

      bool operator!=(const Configuration& rhs) {
        return !(*this == rhs);
      }

    private:
      Parameters params_;
    };

  } // configuration

} // gsoc