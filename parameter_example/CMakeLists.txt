cmake_minimum_required(VERSION 2.8.3)
project(parameter_example)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES parameter_example
  CATKIN_DEPENDS roscpp
  DEPENDS Boost
)

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

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(parameter_example_node src/parameter_example_node.cpp)

target_link_libraries(parameter_example_node
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

catkin_add_gtest(parameter_warehouse_test test/parameter_warehouse_test.cpp)