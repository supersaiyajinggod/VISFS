#include "InterfaceROS.h"

int main(int argc, char ** argv) {
  ros::init(argc, argv, "VISFSInterfaceROSNode");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");

  VISFSInterfaceROS a(n, pnh);

  ros::spin();

  return 0;
}