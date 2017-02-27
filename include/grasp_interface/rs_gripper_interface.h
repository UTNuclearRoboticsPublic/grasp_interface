#include <grasp_interface/gripper_interface.h>

//ROS messages
#include <robotiq_s_model_control/SModel_robot_output.h>
#include <robotiq_s_model_control/SModel_robot_input.h>

#ifndef RS_GRIPPER_INTERFACE_H
#define RS_GRIPPER_INTERFACE_H

/**
 * @brief   Easy interface for working with robotiq gripper.
 * 
 * For details on the interface that this class implements, see these page:
 * http://support.robotiq.com/pages/viewpage.action?pageId=590045 (status)
 * Despite being called "input registers", these are the status messages returned by the gripper.
 * 
 * http://support.robotiq.com/pages/viewpage.action?pageId=590044 (command)
 * These are the bits you set to command the gripper to different positions.
 *
 * Also see the implementations of cb_getGripperStatus and sendCommand, which list the possible values
 * for the inputs and outputs.
 * 
 * 
 * WARNING: This interface does not support scissor mode, so it is disabled for now.
 * TODO To implement scissor mode:
 *   - Determine position limits in scissor mode
 *   - Add functionality to setPosition(single)
 *   - Add warning message to setPosition(all three), then call setPosition(single) with first finger position
 *   - Remove hard block/error message for scissor mode in setMode(enum)
 *   - Implement fullClose() for scissor
 *   - Add test cases to rs_gripper_interface_test.cpp
 *
 * @version 1.0
 * 
 * @author  Adam Allevato <adam.d.allevato@gmail.com>
 * @author  Matt Horn <mwhorn@utexas.edu>
 * @copyright BSD 3-paragraph
 * @date    Nov 23, 2015
 */
class RSGripperInterface : public GripperInterface {
public:
  /**
   * Modes as defined by Robotiq
   */
  enum Mode {
    MODE_BASIC = 0,
    MODE_PINCH,
    MODE_WIDE,
    MODE_SCISSOR
  };
  
  ///List of all finger link names. Used when specifying collision-allowed links for grasping.
  static std::vector<std::string> fingerNames;
  
  ///Default constructor
  ///@arg timeout: Time to wait before giving up on a connection.
  ///@arg sim: if true, connect in simulation mode (don't wait for response from hardware)
  RSGripperInterface(bool sim = false);
  
  // ROS callbacks
  void cb_getGripperStatus(const robotiq_s_model_control::SModel_robot_input& msg);
  void cb_command(const grasp_interface::RSGripperCommand& alpha);
  
  // Basic Commands //////////////////////////////////////////////////////////////////////////
  void reset() override;
  void deactivate() override;
  void activate() override;
  void eStop() override;
  
  // Adv Commands //////////////////////////////////////////////////////////////////////////
  ///Go to basic mode, fully open, medium force, full speed
  void home() override;
  
  /**
   * Fully open in whatever the current mode is
   * WARNING: scissor mode is not implemented
   */
  void fullOpen() override;
  
  /**
   * Fully close in whatever the current mode is
   * WARNING: scissor mode is not implemented
   */
  void fullClose() override;
  
  // Setters //////////////////////////////////////////////////////////////////////////
  
  /**
   * Set the gripper mode and move to the open position for that mode.
   * When new modes are sent to the S-Model, it automatically goes to the open position.
   * This behavior cannot be supressed.
   */
  void setMode(Mode newMode);
  
  /**
   * Set all fingers to the same position. 0 is open, 255 is closed in basic mode, 107 is closed in pinch mode.
   * WARNING: scissor mode is not implemented
   */
  void setPosition(int position);
  
  /// Set speed of all fingers.
  void setSpeed(int speed) override;
  
  
  /// Unique to the 3-finger gripper: set the positions of each finger individually.
  void setPosition(int positionA, int positionB, int positionC);
  
  /// Control the force of the grip.
  void setForce(int force);
  
  // Getters //////////////////////////////////////////////////////////////////////////
  
  /**
   * Robotiq S-Model has built-in sensors to detect whether or not the fingers closed
   * fully, or if they were stopped by an object. This method returns whether or not
   * the fingers closed as expected.
   */
  bool isObjectHeld();
  
  inline bool isActivated() { return activated; };
  
  
private:
  /**
   * Return whether or not the gripper is connected. If it's not, a warning is issued via ROS_WARN.
   */
  bool isConnected();
  
  /// Clamp the passed value to 0-255 inclusive. The name is included for debug purposes.
  void clampByte(int& toClamp, std::string name = "[no name given]");
  
  /// Override to send specific S-Model command.
  void sendCommand() override;
  
  /// Internal mode setter (after enum has been verified).
  void setMode(int newMode);
  
  bool sim;                                               /// If true, simulation mode is active
  
  ros::NodeHandle n;
  ros::AsyncSpinner spinner;
  
  ros::Subscriber gripperCommandSub;                      /// Listen for people trying to use this gripper
  
  ros::Subscriber gripperStatusSub;                       /// Listen to the gripper
  robotiq_s_model_control::SModel_robot_input status;     /// The status returned from the gripper
  
  robotiq_s_model_control::SModel_robot_output command;   /// Store the command to send to the gripper
  ros::Publisher gripperCommandPub;                       /// Used to send the command to the gripper
};

#endif
