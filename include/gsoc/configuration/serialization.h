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

#ifndef DYNAMIC_CONFIG_SERIALIZATION_H
#define DYNAMIC_CONFIG_SERIALIZATION_H

#include "ros/ros.h"

namespace gsoc {

  namespace configuration {

    namespace serialization {

      template <typename T>
      void serialize(const T& data, std::vector<uint8_t>& buffer)
      {
        buffer.resize(ros::serialization::serializationLength(data));
        ros::serialization::OStream ostream(&buffer[0], buffer.size());
        ros::serialization::serialize(ostream, data);
      }

      // template <>
      // void serialize(const std::vector<uint8_t>& data, std::vector<uint8_t>& buffer)
      // { buffer = data; }

      template <typename T>
      std::vector<uint8_t> serialize(const T& data)
      {
        std::vector<uint8_t> buffer;
        serialize(data, buffer);
        return buffer;
      }

      template < typename T >
      void deserialize(std::vector<uint8_t>& data, T& output)
      {
        ros::serialization::IStream istream(&data[0], data.size());
        ros::serialization::Serializer<T>::read(istream, output);
      }

      // template <>
      // void deserialize<std::vector<uint8_t> >(std::vector<uint8_t>& data, std::vector<uint8_t>& output)
      // { 
      //   output = data;
      // }

      template <typename T>
      T deserialize(std::vector<uint8_t> data)
      {
        T output;
        deserialize(data, output);
        return output;
      }

    } // serialization

  } // configuration

} // gsoc

#endif
