#include "gripper_interface.h"


GripperInterface::GripperInterface() :
  block(true),
  connected(true),
  activated(false),
  messagesReceived_(0) {
}


void GripperInterface::setBlocking(bool blocking)
{
  block = blocking;
}
