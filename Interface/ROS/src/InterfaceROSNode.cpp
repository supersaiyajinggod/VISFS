#include "InterfaceROS.h"

int main(int argc, char ** argv) {
	ros::init(argc, argv, "VISFSInterfaceROSNode");
	ros::NodeHandle n;
	ros::NodeHandle pnh("~");
	ros::Rate loopRate(100);

	VISFSInterfaceROS a(n, pnh);

	while(ros::ok()) {
		ros::spinOnce();
		a.publishMessage();
		loopRate.sleep();
  	}

  return 0;
}