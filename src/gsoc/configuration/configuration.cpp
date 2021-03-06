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

#include "gsoc/configuration/configuration.h"

namespace gsoc {

  namespace configuration {

    void Configuration::insert(const std::string& name, const std::string& value) {
      if (insert<bool>(name, value)) return;
      if (insert<std::string>(name, value)) return;
      if (insert<int>(name, value)) return;
      if (insert<long>(name, value)) return;
      if (insert<float>(name, value)) return;
      if (insert<double>(name, value)) return;
    }

    void Configuration::insert(const std::string& line) {
      int pos = line.find("=");
      if (pos > (unsigned int)0 && pos < line.length())
        insert(line.substr(0, pos), line.substr(pos+1, line.length())); 
    }


    bool Configuration::has(const std::string& name) const {
      Parameters::const_iterator it = params_.find(name);
      return notEnd(it);
    }

    int Configuration::size() const {
      return params_.size();
    }

    bool Configuration::equivalent(const Configuration& conf) const {
      return params_.size() == conf.params_.size() &&
             std::equal(params_.begin(), params_.end(), conf.params_.begin(), sameName<Parameter>) &&
             std::equal(params_.begin(), params_.end(), conf.params_.begin(), sameType<Parameter>);
    }

    bool Configuration::operator==(const Configuration& rhs) const {
      return equivalent(rhs) &&
             std::equal(params_.begin(), params_.end(), rhs.params_.begin(), sameValue<Parameter>);
    }

    bool Configuration::operator!=(const Configuration& rhs) const {
      return !(*this == rhs);
    }

    bool Configuration::notEnd(Parameters::const_iterator& it) const {
      return it != params_.end();
    }

  } // configuration

} // gsoc