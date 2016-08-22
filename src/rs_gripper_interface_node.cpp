#include <rs_gripper_interface.h>

int main(int argc, char **argv)
{
  // Initialize the ros grab_interface_node
  ros::init(argc, argv, "rs_gripper_interface_node");
  
  if(argc < 2) { 
    ROS_ERROR("usage is: rosrun rs_gripper_interface_node ['sim']");
    return -1;
  }
  
  bool sim = false;
  if(argc > 2) {
    sim = (argv[2] == "sim");
  }
  
  RSGripperInterface r = RSGripperInterface(sim);

  ros::waitForShutdown();
  return 0;
}
