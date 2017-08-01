cmake_minimum_required(VERSION 2.8.3)
project(grasp_interface)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  robotiq_c_model_control
  robotiq_s_model_control
  roscpp
  std_msgs
)

add_message_files(
  FILES
  RCGripperCommand.msg
  RSGripperCommand.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    robotiq_c_model_control
    robotiq_s_model_control
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(${PROJECT_NAME} src/gripper_interface.cpp src/rs_gripper_interface.cpp src/rc_gripper_interface.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(rs_gripper_interface_node src/gripper_interface.cpp  src/rs_gripper_interface_node.cpp)
add_dependencies(rs_gripper_interface_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(rs_gripper_interface_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(rs_gripper_interface_test src/rs_gripper_interface_test.cpp)
add_dependencies(rs_gripper_interface_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(rs_gripper_interface_test ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(rc_gripper_interface_node src/gripper_interface.cpp  src/rc_gripper_interface_node.cpp)
add_dependencies(rc_gripper_interface_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  grasp_interface_generate_messages_cpp)
target_link_libraries(rc_gripper_interface_node ${catkin_LIBRARIES} ${PROJECT_NAME})


install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
