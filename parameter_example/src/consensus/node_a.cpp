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

#include "ros/ros.h"
#include "parameter_server/parameter_server.h"
#include "ros_parameter/parameter_server_listener.h"
#include "ros_parameter/parameter_server_client.h"

typedef std::vector<uint8_t> Data;

struct OnUpdateNotification {
  std::string name;
  Data data;
};

OnUpdateNotification on_update_handler_notification;
void on_update_handler(const std::string& name, const Data& data)
{
  on_update_handler_notification.name = name;
  on_update_handler_notification.data = data;
}

bool on_change_handler(const std::string& name, const Data& data)
{
  return data.size() < 30;
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "node_a");

  parameter_server::ParameterServer parameterServer( ros::NodeHandle("/parameters_server") );

  ros_parameter::ParameterServerListener paramServerListener;
  paramServerListener.on_update( on_update_handler );
  paramServerListener.on_change( on_change_handler );

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();

  Data allOnes_10(10, 1);
  Data allOnes_20(20, 1);
  Data allOnes_30(30, 1);
  Data allTwos_10(10, 2);

  using namespace ros_parameter;

  // Get a value that doesn't exist yet should return the default value
  {
    Data ret_data;
    ROS_ASSERT( get_parameter("/allOnes", allOnes_10, ret_data) );
    ROS_ASSERT( allOnes_10 == ret_data );
  }

  // Get of an initialize parameter should return the initialized data
  {
    Data ret_data;
    ROS_ASSERT( set_parameter("/allTwos", allTwos_10, "") );
    ROS_ASSERT( get_parameter("/allTwos", allOnes_10, ret_data) );
    ROS_ASSERT( allTwos_10 == ret_data );
  }

  // Setting data should notify listeners with the new data
  ROS_ASSERT( add_on_update_parameter_listener("/allOnes", paramServerListener.on_update_service()) );    
  ROS_ASSERT( set_parameter("/allOnes", allOnes_20, "") );
  ROS_ASSERT( "/allOnes" == on_update_handler_notification.name );
  ROS_ASSERT( allOnes_20 == on_update_handler_notification.data );

  // Setting data should notify listeners except the service specified
  ROS_ASSERT( set_parameter("/allOnes", allOnes_10, paramServerListener.on_update_service()) );
  ROS_ASSERT( "/allOnes" == on_update_handler_notification.name );
  ROS_ASSERT( allOnes_20 == on_update_handler_notification.data );

  // Unsubscribe update listener. on_update_handler_notification should remain the same
  ROS_ASSERT( remove_on_update_parameter_listener("/allOnes", paramServerListener.on_update_service()) );
  ROS_ASSERT( set_parameter("/allOnes", allOnes_10, "") );
  ROS_ASSERT( "/allOnes" == on_update_handler_notification.name );
  ROS_ASSERT( allOnes_20 == on_update_handler_notification.data );

  // Should reject data if it is not valid
  ROS_ASSERT( add_on_change_parameter_listener("/allOnes", paramServerListener.on_change_service()) );
  ROS_ASSERT( !set_parameter("/allOnes", allOnes_30, "") );

  // Should valid data if no on change listeners
  ROS_ASSERT( remove_on_change_parameter_listener("/allOnes", paramServerListener.on_change_service()) );
  ROS_ASSERT( set_parameter("/allOnes", allOnes_30, "") );

  ROS_INFO("Test finish correcty!!");
}