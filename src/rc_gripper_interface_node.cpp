#include <rc_gripper_interface.h>

int main(int argc, char **argv)
{
  // Initialize the ros grab_interface_node
  ros::init(argc, argv, "rc_gripper_interface_node");
  
  RCGripperInterface r = RCGripperInterface();

  ros::waitForShutdown();
  return 0;
}
