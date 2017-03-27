# Overview

A higher-level wrapper for the Robotiq S and C models.
The package is designed as a replacement and/or upgrade for the ROS-Industrial robotiq controller nodes.
For the S-Model gripper, this package also will publish joint states and TF frames based on the reported
positions of the gripper fingers, allowing full visualization in RViz. This is not yet 

Written in C++, it can function as either a pure C++ class (use function calls to open/close the gripper),
 or as a full ROS node (use ROS communication to control the gripper, works with Python or C++).

# Using the ROS nodes
Regardless of whether you plan on controlling the gripper via the C++ or ROS interfaces, several other nodes must be run
to successfully connect to the gripper. Run the following launch file to control the gripper:
```
# 3-Finger Adaptive (S-Model) Gripper
roslaunch grasp_interface rs_gripper.launch

# 2-Finger 85/140 (C-Model) Gripper
roslaunch grasp_interface rc_gripper.launch
```

## Parameters
**sim**: if true, then a simulated robotiq dummy interface will be launched along with the control interface.
The simulated C-Model robot is currently not implemented.

**node**: if true, then a ROS node will launch which will allow control of the gripper via ROS calls.

**ip**: the IP address of the gripper you want to connect to. Not used if **sim** is true.

**robotiq_prefix**: A ROS tf prefix that will be prepended to the frame names. Used by the joint state publisher.
