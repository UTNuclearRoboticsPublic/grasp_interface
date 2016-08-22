#include <math.h>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <grasp_interface/RSGripperCommand.h>
#include <grasp_interface/RCGripperCommand.h>

#ifndef GRIPPER_INTERFACE_H
#define GRIPPER_INTERFACE_H

/**
 * @brief   Easy interface for working with arbitrary gripper
 * 		2 methods to use it:
 *		1) C++ objects, as seen in rs_gripper_interface_test.cpp
 *		2) Publish commands on the ROS topic "grip_command".
 * @version 1.0
 * 
 * @author  Adam Allevato <adam.d.allevato@gmail.com>
 * @copyright BSD 3-paragraph
 * @date    Dec 4, 2015
 */
class GripperInterface {
public:
  GripperInterface();
  // Basic Commands //////////////////////////////////////////////////////////////////////
  
  /// After calling this, the gripper's internal state should be reset, including clearing any faults.
  virtual void reset() {};
  /**
   * Shut down the gripper. activate() should have to be called after this if the
   * caller wants to do anything with the gripper.
   */
  virtual void deactivate() = 0;
  /// Prepare the gripper for motion. Should require call before any motion occurs.
  virtual void activate() = 0;
  /// Stop the robot as quickly as possible.
  virtual void eStop() = 0;
  
  // Adv Commands //////////////////////////////////////////////////////////////////////
  
  /// Move the robot to a generally "decent" starting position.
  virtual void home() = 0;
  
  /// Fully close the gripper
  virtual void fullOpen() = 0;
  /// Fully open the gripper
  virtual void fullClose() = 0;
  
  // Setters //////////////////////////////////////////////////////////////////////
  
  /**
   * Set the position of the fingers. 0 represents fully open, with higher numbers being
   * more and more closed. Units and range will depend on specific gripper.
   */
  void setPosition(int position) {};
  /**
   * The speed at which the gripper moves. 0 is slowest. Units and range will depend on specific gripper.
   */
  virtual void setSpeed(int speed) = 0;
  /**
   * Whether or not calls to this interface should block until the hardware is complete.
   * Grippers that are open-loop should override this method to disable blocking functionality.
   */
  void setBlocking(bool blocking);
  
protected:
  /**
   * Convenience: clamp a number between 0 and 255.
   */
  void clampByte(int& toClamp, std::string name) {};
  
  bool block;          /// If true, commands wait for closed loop completion before returning
  bool connected;
  bool activated;     /// If true, the gripper is ready to move.

  int messagesReceived_; /// The number of status messages received from the gripper since node begin. Can be used to check if gripper is connected
  
private:
  /**
   * Apply changes to the hardware. Implement as empty for a simulated gripper.
   */
  virtual void sendCommand() = 0;
};

#endif
