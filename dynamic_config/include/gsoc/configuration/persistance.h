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

#ifndef DYNAMIC_CONFIG_PERSISTANCE_H
#define DYNAMIC_CONFIG_PERSISTANCE_H

#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>

namespace gsoc {

  namespace configuration {

    namespace persistance {

      template <typename T>
      bool isType(const std::string& s) {
        return false;
      }

      template <> bool isType<bool>(const std::string& s);

      template <> bool isType<std::string>(const std::string& s);

      template <> bool isType<int>(const std::string& s);

      template <> bool isType<long>(const std::string& s);

      template <> bool isType<float>(const std::string& s);
      
      template <> bool isType<double>(const std::string& s);

      template <typename T>
      std::string trim(const std::string& s) {
        return "";
      }

      template <> std::string trim<bool>(const std::string& s);

      template <> std::string trim<std::string>(const std::string& s);

      template <> std::string trim<int>(const std::string& s);

      template <> std::string trim<long>(const std::string& s);

      template <> std::string trim<float>(const std::string& s);

      template <> std::string trim<double>(const std::string& s);

      template <typename T>
      T cast(const std::string& value) {
        return boost::lexical_cast<T>(trim<T>(value));
      }

      template <> bool cast(const std::string& value);

      template <typename T>
      std::string toString(const T& t) {
        try {
          return boost::lexical_cast<std::string>(t);
        } catch (boost::bad_lexical_cast &) {
          return "unknown";
        }
      }

      template <> std::string toString(const std::string& s);

      template <> std::string toString(const float& f);

      template <> std::string toString(const bool& b);

      template <> std::string toString(const long& l);

      struct ToString {

        template <typename T>
        std::string operator()(std::pair<std::string,T> pair) const {
          std::string out = pair.first;
          out += "=";
          out += toString(pair.second);
          return out;
        }

      };

    } // persistance

  } // configuration

} // gsoc

#endif