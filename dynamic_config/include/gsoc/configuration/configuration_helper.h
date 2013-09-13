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

#ifndef DYNAMIC_CONFIG_CONFIGURATION_HELPER_H
#define DYNAMIC_CONFIG_CONFIGURATION_HELPER_H
 
namespace gsoc {

  namespace configuration {

    namespace {

      template <typename Result, typename Visitor>
      struct  UnaryOperation : boost::static_visitor<Result> {

        UnaryOperation(const std::string& name, const Visitor& visitor)
        : name_(name)
        , visitor_(visitor)
        { }

        template <typename T> 
        Result operator()(const T& t) const {
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

        template <typename Param>
        Result operator()(const std::pair<const std::string, Param>& pair) const {
          UnaryOperation<Result,Visitor> op(pair.first, visitor_);
          return boost::apply_visitor(op, pair.second);
        }

        const Visitor& visitor_;
      };

      template <typename Result, typename Visitor>
      Operation<Result,Visitor> make_operation(const Visitor& visitor) {
        return Operation<Result,Visitor>(visitor);
      }

      template <typename Param>
      std::string parameterName(const std::pair<const std::string, Param>& pair) {
        return pair.first;
      }

      template <typename Param>
      bool sameName(std::pair<const std::string, Param> p1,
                    std::pair<const std::string, Param> p2) {
        return p1.first == p2.first;
      }

      template <typename Param>
      bool sameType(std::pair<const std::string, Param> p1,
                    std::pair<const std::string, Param> p2) {
        return p1.second.type() == p2.second.type();
      }

      template <typename Param>
      bool sameValue(std::pair<const std::string, Param> p1,
                     std::pair<const std::string, Param> p2) {
        return p1.second == p2.second;
      }

    } // anonymous

  } // configuration

} // gsoc

#endif
