
#include "VrpnAgentInterface.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vrpninterface_node");	// initializes Node Name

	std::cout<<"Node name : "<<ros::this_node::getName()<<std::endl;
	std::vector<double> borderx;
	std::vector<double> bordery;
	borderx.push_back(0);
	bordery.push_back(0);
	borderx.push_back(1);
	bordery.push_back(0);
	borderx.push_back(1);
	bordery.push_back(1);
	borderx.push_back(0);
	bordery.push_back(1);
	
	VrpnAgentInterface va(5, borderx, bordery, 20);

	return 0;
}
