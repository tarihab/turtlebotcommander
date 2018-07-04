
#include "VrpnAgentInterface.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vrpninterface_node");	// initializes Node Name

	std::cout<<"Node name : "<<ros::this_node::getName()<<std::endl;

	//VrpnAgentInterface va(nagents, borderx, bordery, looprate);
	// argument to constructor is boolean whether to publish the data
	// or not, and it should normally be true
	VrpnAgentInterface va(true);

	return 0;
}
