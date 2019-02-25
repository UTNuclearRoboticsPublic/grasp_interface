// Copyright (c) 2016, The University of Texas at Austin
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "grasp_interface/r2f_gripper_interface.h"


r2fGripperInterface::r2fGripperInterface() :
  n(),
  spinner(2),
  command()
{
    spinner.start();
    // In case commands are sent via a ROS topic:
    gripperCommandSub = n.subscribe("grip_command",1, &r2fGripperInterface::cb_command,this);    
    
    gripperCommandPub = n.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("2fModelRobotOutput",1);
    gripperStatusSub = n.subscribe("2fModelRobotInput",1,&r2fGripperInterface::cb_getGripperStatus,this);
    
    float printTime = 10, retryTime = 0.1;
    
    ros::Time start = ros::Time::now();
    while(
      (gripperCommandPub.getNumSubscribers() <= 0 || status.gPO == 0) && // wait for connection
      !ros::isShuttingDown())                                             // but stop if ROS shuts down
    {
      ROS_INFO_STREAM_THROTTLE(printTime, "[r2fGripperInterface] Waiting for connection with gripper (" << (ros::Time::now() - start) << "s elasped)");
      ros::Duration(retryTime).sleep();
    }
    if(!ros::isShuttingDown()) {
      ROS_INFO("[r2fGripperInterface] Connected to gripper");
      connected = true;
      
      //check for pre-activation
      if(status.gSTA == 3) {
	activated = true;
	command.rPR = status.gPO;
	command.rSP = 255;
  	command.rFR = 255;
	ROS_DEBUG_STREAM("[r2fGripperInterface] Gripper already activated");
      }
    }
}

void r2fGripperInterface::sendCommand() {
  if(!r2fGripperInterface::isConnected()) {
    ROS_ERROR("[r2fGripperInterface] Can't control the gripper, it's not connected or activated yet.");
    return;
  }

  if(!activated) {
    ROS_ERROR("[r2fGripperInterface] Can't control the gripper, it's not activated yet. Call activate()");
    return;
  }
  
  gripperCommandPub.publish(command);
  return;
}

void r2fGripperInterface::eStop() {
  ROS_FATAL("[r2fGripperInterface] Beginning soft RS gripper E-stop");
  command.rATR = 1;
  sendCommand();
  if(block) {
    while(status.gFLT < 13) {
      ROS_WARN_DELAYED_THROTTLE(1, "Waiting for E-Stop to complete");
    }
  }
  ROS_FATAL("r2fGripperInterface] Finished soft r2f gripper E-stop. Reactivate to start again");
}


void r2fGripperInterface::reset()
{
  ROS_DEBUG_STREAM("r2fGripperInterface] Beginning reset");
  deactivate();
  command = robotiq_2f_gripper_control::Robotiq2FGripper_robot_output();
  activate();
  ROS_DEBUG_STREAM("r2fGripperInterface] Finished reset");
}

void r2fGripperInterface::deactivate()
{
  ROS_DEBUG_STREAM("r2fGripperInterface] Deactivating");
  command.rACT = 0;
  sendCommand();
  // The following needs to be updated for a 2f-model gripper
/*
  if(block) {
    while(status.gACT != 0) {
      ROS_WARN_DELAYED_THROTTLE(5, "r2fGripperInterface] Waiting for gripper to turn off...");
    }
    ROS_DEBUG_STREAM("[r2fGripperInterface] Finished dectivation");
  }
*/
  activated = false;
}

void r2fGripperInterface::activate()
{
  //check for pre-activation
  if(status.gSTA == 3) {
    activated = true;
    command.rPR = status.gPO;
    command.rSP = 255;
    command.rFR = 255;
    ROS_DEBUG_STREAM("[r2fGripperInterface] Gripper already activated");
    return;
  }

  ROS_DEBUG_STREAM("[r2fGripperInterface] Activating");

  command.rACT = 1;

  //don't call sendCommand, because sendCommand includes an activation check. Instead
  // just manually send the command
  gripperCommandPub.publish(command);
  while(status.gSTA != 3) {
    ROS_INFO_DELAYED_THROTTLE(10, "[RSGripperInterface] Waiting for activation to complete...");
  }
  ROS_DEBUG_STREAM("[RSGripperInterface] Finished activation");

  activated = true;
}

void r2fGripperInterface::home()
{
  setSpeed(255);
  setForce(128);
  setPosition(0);
}

void r2fGripperInterface::fullOpen()
{
  ROS_DEBUG_STREAM("[r2fGripperInterface] Opening gripper");
  setPosition(0);
}

void r2fGripperInterface::fullClose()
{
  ROS_DEBUG_STREAM("[r2fGripperInterface] Closing gripper");
  setPosition(255);
}


void r2fGripperInterface::setPosition(int position) {

  ROS_DEBUG_STREAM("[r2fGripperInterface] Moving fingers to position " << position);
 
  command.rPR = position;
  command.rGTO = 1;
  sendCommand();
  
  // The following was copied from the Model S. No analog for the Model C?:

/*
  //if(block) {
    if(status.gPR != position) {
      while(status.gGTO != 0) {
	ROS_INFO_THROTTLE(5, "[r2fGripperInterface] Waiting for move to begin...");
      }
    }
    while(status.gGTO) {
      ROS_INFO_THROTTLE(5, "[r2fGripperInterface] Waiting for move to complete...");
    }
  //}
*/

}

void r2fGripperInterface::setSpeed(int speed)
{

  ROS_DEBUG_STREAM("[r2fGripperInterface] Setting fingers to speed " << speed);
  command.rSP = speed;
}

void r2fGripperInterface::setForce(int force)
{

  ROS_DEBUG_STREAM("[r2fGripperInterface] Setting fingers to force " << force);
  command.rFR = force;
}

void r2fGripperInterface::cb_getGripperStatus(const robotiq_2f_gripper_control::Robotiq2FGripper_robot_input& msg)
{
  status = msg;
  
  switch(status.gFLT) {
    case 0:
      break;
    case 5:
      ROS_WARN_THROTTLE(1, "[r2fGripperInterface] Activation is not complete!");
      break;
    case 6:
      ROS_WARN_THROTTLE(1, "[r2fGripperInterface] Mode change is not complete!");
      break;
    case 7:
      ROS_WARN_THROTTLE(1, "[r2fGripperInterface] Gripper is not activated yet!");
      break;
    case 9:
    case 10:
    case 11:
      ROS_ERROR_STREAM_THROTTLE(10, "[r2fGripperInterface] Minor fault detected, #" << (int)status.gFLT);
      break;
    case 13:
    case 14:
    case 15:
      ROS_FATAL_STREAM_THROTTLE(10, "[r2fGripperInterface] Major fault detected, #" << (int)status.gFLT);
      break;
    default:
      break;
  }
}

// Send commands via a ROS topic, as opposed to the other methods.
void r2fGripperInterface::cb_command(const grasp_interface::r2fGripperCommand& alpha)
{
  if (activated != true)
	activate();
  
  //order is important
  setForce(alpha.force);
  setPosition(alpha.position);
  setSpeed(alpha.speed);
  
  //sendCommand();
  gripperCommandPub.publish(command);

  ROS_DEBUG_STREAM("Moving");
}

bool r2fGripperInterface::isConnected() {
  if(!connected) {
    ROS_WARN_THROTTLE(2.0, "[r2fGripperInterface]: Not connected!");
    return false;
  }
  return true;
}
