cmake_minimum_required(VERSION 2.8.3)
project(dynamic_config)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation)
find_package(Boost REQUIRED COMPONENTS system)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  Param.msg
  Conf.msg
)

add_service_files(
  FILES
  GetConf.srv
  SetConf.srv
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamic_config
  CATKIN_DEPENDS roscpp
  DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(dynamic_config 
    src/gsoc/configuration/configuration.cpp
    src/gsoc/configuration/configuration_builder.cpp
    src/gsoc/configuration/configuration_listener.cpp
    src/gsoc/configuration/configuration_client.cpp
    src/gsoc/configuration/configuration_server.cpp
    src/gsoc/configuration/persistance.cpp)

add_executable(config_prototype_node src/config_prototype_node.cpp)
add_executable(reconfigurable_node src/dynamic_config/reconfigurable_node.cpp)
add_executable(client_node src/dynamic_config/client_node.cpp)
add_executable(listener_node src/dynamic_config/listener_node.cpp)

target_link_libraries(config_prototype_node
  ${catkin_LIBRARIES}
  dynamic_config
)

target_link_libraries(reconfigurable_node
  ${catkin_LIBRARIES}
  dynamic_config
)

target_link_libraries(client_node
  ${catkin_LIBRARIES}
  dynamic_config
)

target_link_libraries(listener_node
  ${catkin_LIBRARIES}
  dynamic_config
)