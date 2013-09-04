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
#include <algorithm>
#include <boost/variant.hpp>
#include <boost/bind.hpp>

namespace gsoc {

  namespace configuration {

    namespace {

      typedef boost::variant<std::string, int> Parameter;
      typedef std::map<std::string, Parameter> Parameters;

      template <typename Visitor>
      struct VoidVisitor : boost::static_visitor<> {
        Visitor visitor;
        std::string name;
        template <typename T>
        void operator()(T& t) const
        { visitor(make_pair(name, t)); }
      };

      template <typename Visitor>
      void make_VoidVisitor(std::pair<const std::string,Parameter> pair, Visitor visitor) {
        VoidVisitor<Visitor> v;
        v.visitor = visitor;
        v.name = pair.first;
        boost::apply_visitor(v, pair.second);
      }

      template <typename ResultType, typename Visitor>
      struct ResultVisitor : boost::static_visitor<ResultType> {
        Visitor visitor;
        std::string name;
        template <typename T> ResultType operator()(T& t) const
        { return visitor(std::make_pair(name, t)); }
      };

      template <typename ResultType, typename Visitor>
      ResultType make_ResultVisitor(std::pair<const std::string, Parameter> pair, Visitor visitor) {
        ResultVisitor<ResultType, Visitor> v;
        v.visitor = visitor;
        v.name = pair.first;
        return boost::apply_visitor(v, pair.second);
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
      T get(const std::string& name) {
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
          make_VoidVisitor(*it, visitor);
        else
          std::cerr << "Param " << name << " does not exist" << std::endl;
      }

      template <typename ResultType, typename Visitor>
      ResultType apply(const std::string& name, Visitor visitor) {
        Parameters::iterator it = params_.find(name);
        if (it != params_.end())
          return make_ResultVisitor<ResultType>(*it, visitor);
        else
          std::cerr << "Param " << name << " does not exist" << std::endl;
        return ResultType();
      }

      template <typename Visitor>
      void applyAll(Visitor visitor) {
        std::for_each(params_.begin(), params_.end(), 
          boost::bind(make_VoidVisitor<Visitor>, _1, visitor));
      }

      template <typename ResultType, typename Visitor, typename OutputIterator>
      void applyAll(Visitor visitor, OutputIterator result) {
        std::transform(params_.begin(), params_.end(), result, 
          boost::bind(make_ResultVisitor<ResultType, Visitor>, _1, visitor));
      }

      bool similar(const Configuration& conf) {
        return params_.size() == conf.params_.size() &&
               std::equal(params_.begin(), params_.end(), conf.params_.begin(), sameName) &&
               std::equal(params_.begin(), params_.end(), conf.params_.begin(), sameType);
      }

      bool operator==(const Configuration& rhs) {
        return similar(rhs) &&
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