cmake_minimum_required(VERSION 2.8.3)
project(parameter_example)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

find_package(Boost REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(		
  FILES
  ParamValue.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetParam.srv
  GetParam.srv
  HasParam.srv
  SearchParam.srv
  DeleteParam.srv
  SubscribeParam.srv
  UnsubscribeParam.srv
  ParamEvent.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES parameter_example
  CATKIN_DEPENDS message_runtime roscpp
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(parameter_example_node src/parameter_example_node.cpp)
target_link_libraries(parameter_example_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(parameter_server src/parameter_server.cpp)
target_link_libraries(parameter_server
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(rosparam_node src/rosparam_node.cpp)
target_link_libraries(rosparam_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(ros_parameter_example_node src/ros_parameter_example_node.cpp)
target_link_libraries(ros_parameter_example_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(ros_parameter_group_example_node src/ros_parameter_group_example_node.cpp)
target_link_libraries(ros_parameter_group_example_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS parameter_example_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/ros/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/ros/
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


# Build tests
catkin_add_gtest(test_command test/command_test.cpp)
catkin_add_gtest(test_commands_queue test/commands_queue_test.cpp)
catkin_add_gtest(test_subscribers_manager test/subscribers_manager_test.cpp)
catkin_add_gtest(test_parameters_container test/parameters_container_test.cpp)
catkin_add_gtest(test_parameter test/parameter_test.cpp)
catkin_add_gtest(test_parameters_server test/parameters_server_test.cpp)

target_link_libraries(test_parameter ${Boost_LIBRARIES} )
