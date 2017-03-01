#include <ros/ros.h>
#include <iostream>
#include <grasp_interface/rs_gripper_interface.h>

/**
 * @brief   Test the robotiq gripper code
 */
 
int main(int argc, char** argv) {
  ros::init(argc, argv, "rs_gripper_interface_test");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  RSGripperInterface gripper = RSGripperInterface(false);
  ROS_INFO("[RSGripperInterfaceTest] activating");
  gripper.activate();
  return 1;
  
  //temp tests/////////////////////////////
  
  
  ros::Duration(2.0).sleep();
  
  //full functionality/////////////////////
  
  
  ROS_INFO("[RSGripperInterfaceTest] resetting");
  gripper.reset();
  
  ROS_INFO("[RSGripperInterfaceTest] testing setMode");
  gripper.setMode(RSGripperInterface::MODE_BASIC);
  ros::Duration(1.0).sleep();
  gripper.setMode(RSGripperInterface::MODE_PINCH);
  ros::Duration(1.0).sleep();
  gripper.setMode(RSGripperInterface::MODE_WIDE);
  ros::Duration(1.0).sleep();
  gripper.setMode(RSGripperInterface::MODE_SCISSOR);
  ros::Duration(1.0).sleep();
  
  ROS_INFO("[RSGripperInterfaceTest] testing setSpeed, fullOpen, fullClose");
  //slow close
  gripper.home();
  gripper.setSpeed(-1);
  gripper.fullClose();
  ros::Duration(1.0).sleep();
  
  //fast open
  gripper.setSpeed(300);
  gripper.fullOpen();
  ros::Duration(1.0).sleep();
  
  //medium close in pinch mode
  gripper.setSpeed(128);
  gripper.setPosition(128);

  gripper.setSpeed(255);
  gripper.setMode(RSGripperInterface::MODE_PINCH);
  gripper.setSpeed(128);
  gripper.setPosition(255, 0, 128);
};
