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
#include <ros_parameter/ros_parameter.h>

std::map<std::string, ros_parameter::ParameterGroup::DataType> g_newData;
std::map<std::string, bool> g_changed;

void on_group_change(const ros_parameter::ParameterGroup &pg, 
                     std::map<std::string, ros_parameter::ParameterGroup::DataType> &newData,
                     std::map<std::string, bool> &changed)
{
  g_newData = newData;
  g_changed = changed;
}

std::map<std::string, bool> g_update_changes;

void on_group_update(ros_parameter::ParameterGroup &pg,
                     std::map<std::string, bool>& changed)
{
  g_update_changes = changed;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "parameter_group_example_node");
  ros::NodeHandle n;

  // Test namespaces
  {
    ros_parameter::ParameterGroup pg1;
    ROS_ASSERT( "/" == pg1.get_namespace() );
 
    ros_parameter::ParameterGroup pg2("/my_namespace");
    ROS_ASSERT( "/my_namespace" == pg2.get_namespace() );
  }

  // Test copy constructor and operator=
  {
    ros_parameter::ParameterGroup pg1;
    ros_parameter::ParameterGroup pg2(pg1);
    ros_parameter::ParameterGroup pg3 = pg1;
    ROS_ASSERT( pg1 == pg2 );
    ROS_ASSERT( pg2 == pg3 );
    ROS_ASSERT( !(pg1 != pg2) );
    ROS_ASSERT( !(pg2 != pg3) );
  }

  // Test parameter name
  {
    ros_parameter::ParameterGroup pg;
    ros_parameter::Parameter<std::string> msg1 = pg.create_parameter("msg1", std::string("Hello"));
    ROS_ASSERT( "/msg1" == msg1.name() );
  } 
  {
    ros_parameter::ParameterGroup pg("/my_namespace");
    ros_parameter::Parameter<std::string> msg1 = pg.create_parameter("msg1", std::string("Hello"));
    ROS_ASSERT( "/my_namespace/msg1" == msg1.name() );
  }

  // Test callbacks
  {
    ros_parameter::ParameterGroup pg;
    pg.on_change(on_group_change);
    pg.on_update(on_group_update);
    ros_parameter::Parameter<std::string> msg1 = pg.create_parameter("msg1", std::string("Hello"));
    ros_parameter::Parameter<std::string> msg2 = pg.create_parameter("msg2", std::string("World!"));
    ros_parameter::Parameter<int> int1 = pg.create_parameter("int1", 100);

    msg1.data("Hola");
    ROS_ASSERT( g_changed[ msg1.name() ] );
    ROS_ASSERT( !g_changed[ msg2.name() ] );
    ROS_ASSERT( !g_changed[ int1.name() ] );

    ROS_ASSERT( g_update_changes[ msg1.name() ] );
    ROS_ASSERT( !g_update_changes[ msg2.name() ] );
    ROS_ASSERT( !g_update_changes[ int1.name() ] );

    ROS_ASSERT( "Hola" == msg1.data() );
  }

  ROS_INFO("Test finish satisfactory!!");
}