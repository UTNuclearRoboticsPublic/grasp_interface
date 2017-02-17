cmake_minimum_required(VERSION 2.8.3)
project(grasp_interface)

set(GCC_COVERAGE_COMPILE_FLAGS "-g -Wno-enum-compare -std=c++11 -Wenum-compare -Wformat-security")
set(GCC_COVERAGE_LINK_FLAGS    "-Wformat-security")

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
set( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )

find_package(catkin REQUIRED COMPONENTS
  robotiq_s_model_control
  robotiq_c_model_control
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  obj_interface
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  RSGripperCommand.msg
  RCGripperCommand.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs obj_interface
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grasp_interface
  CATKIN_DEPENDS obj_interface robotiq_s_model_control robotiq_c_model_control
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

# Declare a C++ library
add_library(grasp_interface src/gripper_interface.cpp src/rs_gripper_interface.cpp src/rc_gripper_interface.cpp)
add_dependencies(grasp_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} grasp_interface_generate_messages_cpp)
target_link_libraries(grasp_interface ${catkin_LIBRARIES})

add_executable(rs_gripper_interface_node src/gripper_interface.cpp  src/rs_gripper_interface_node.cpp)
add_dependencies(rs_gripper_interface_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS} 
  grasp_interface
  grasp_interface_generate_messages_cpp)
target_link_libraries(rs_gripper_interface_node ${catkin_LIBRARIES} grasp_interface)

add_executable(rs_gripper_interface_test src/rs_gripper_interface_test.cpp)
add_dependencies(rs_gripper_interface_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} grasp_interface grasp_interface_generate_messages_cpp)
target_link_libraries(rs_gripper_interface_test ${catkin_LIBRARIES} grasp_interface)

add_executable(rc_gripper_interface_node src/gripper_interface.cpp  src/rc_gripper_interface_node.cpp)
add_dependencies(rc_gripper_interface_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS} 
  grasp_interface 
  grasp_interface_generate_messages_cpp)
target_link_libraries(rc_gripper_interface_node ${catkin_LIBRARIES} grasp_interface)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
