#include <jog_arm/jog_arm_server.h>

static const char* const NODE_NAME = "jog_arm_server";

int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE_NAME);
  jog_arm::JogROSInterface ros_interface(NODE_NAME);

  return 0;
}