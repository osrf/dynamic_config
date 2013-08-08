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

#include <ros/ros.h>
#include "gsoc/param_server.h" 
#include "gsoc/param.h"

// data
std::vector<uint8_t> tenOnes(10, 1);
std::vector<uint8_t> tenTwos(10, 2);

bool update_parameter_callback_called = false;
void update_parameter_callback(const std::vector<uint8_t>& old, const std::vector<uint8_t>& actual)
{
  ROS_ASSERT(old == tenOnes);
  ROS_ASSERT(actual == tenTwos);
  update_parameter_callback_called = true;  
}

bool accept_parameter_callback(const std::vector<uint8_t>& old, const std::vector<uint8_t>& actual)
{
  ROS_ASSERT(old == tenOnes);
  ROS_ASSERT(actual == tenTwos);
  return true;
}

bool do_not_accept_parameter_callback(const std::vector<uint8_t>& old, const std::vector<uint8_t>& actual)
{
  ROS_ASSERT(old == tenOnes);
  ROS_ASSERT(actual == tenTwos);
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "parameter_server_node_test");
  gsoc::param::init();

  gsoc::param_server::init("param_server");
  ros::AsyncSpinner spinner(0);
  spinner.start();

  // Set a parameter
  ROS_ASSERT(gsoc::param::set("param1", tenOnes));
  ROS_ASSERT(gsoc::param::has("param1"));

  // Get existent parameter
  {
    std::vector<uint8_t> result;
    ROS_ASSERT(gsoc::param::get("param1", result));
    ROS_ASSERT( tenOnes == result );
    ROS_ASSERT(!gsoc::param::get("param2", result));
  }

  // Delete a parameter
  ROS_ASSERT(gsoc::param::del("param1"));
  ROS_ASSERT(!gsoc::param::has("param1"));

  // Delete a non existent parameter
  ROS_ASSERT(!gsoc::param::del("param1"));

  // Add an updater and set a new data
  {
    update_parameter_callback_called = false;
    ROS_ASSERT(gsoc::param::set("param1", tenOnes));
    ROS_ASSERT(gsoc::param::addUpdater("param1", update_parameter_callback));
    ROS_ASSERT(gsoc::param::set("param1", tenTwos));
    ROS_ASSERT(update_parameter_callback_called);

    // Remove an updater
    update_parameter_callback_called = false;
    ROS_ASSERT(gsoc::param::removeUpdater("param1"));
    ROS_ASSERT(gsoc::param::set("param1", tenOnes));
    ROS_ASSERT(!update_parameter_callback_called);
  }

  // Add an acceptor
  {
    std::vector<uint8_t> result;
    ros::NodeHandle n("~");
    ROS_ASSERT(gsoc::param::set("param1", tenOnes));
    ROS_ASSERT(gsoc::param::addAcceptor("param1", accept_parameter_callback));
    ROS_ASSERT(gsoc::param::set("param1", tenTwos));
    ROS_ASSERT(gsoc::param::get("param1", result));
    ROS_ASSERT(tenTwos == result);

    // Adding a second callback should fail
    ROS_ASSERT(!gsoc::param::addAcceptor("param1", do_not_accept_parameter_callback));

    // Remove the callback
    ROS_ASSERT(gsoc::param::removeAcceptor("param1"));

    // Remove again the callback should fail
    ROS_ASSERT(!gsoc::param::removeAcceptor("param1"));

    // Add not acceptant callback
    ROS_ASSERT(gsoc::param::set("param1", tenOnes));
    ROS_ASSERT(gsoc::param::addAcceptor("param1", do_not_accept_parameter_callback));
    ROS_ASSERT(!gsoc::param::set("param1", tenTwos));
    ROS_ASSERT(gsoc::param::get("param1", result));
    ROS_ASSERT(tenOnes == result);
  }

  ROS_INFO("Test finished ok!");
}
