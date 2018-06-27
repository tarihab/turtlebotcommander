#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>

#include "voronoi_distributed.h"
#include <turtlebotcommander/Polygonvert2d.h>

#include <iostream>
#include <vector>
#include <cmath>
#include <string>
//#include <sstream>

// class VrpnAgentInterface;

// void cbVrpndataUpdate(const geometry_msgs::TransformStamped::ConstPtr&, int, VrpnAgentInterface*);

class VrpnAgentInterface
{

	private:
		ros::NodeHandle nh;
		ros::Publisher voronoidata_pub;
		std::vector<ros::Subscriber> vrpndata_sub;		 	

		int myid;  // the id of the agent
		int n;  // no.of agents (required to specify the length of the vectors below)
		bool publish_data;  // whether the computed data are to be published.. by default true
		bool vrpn_updated;
		std::vector<double> xsites;  // the x-positions of all agents (required for computing voronoi partition)
		std::vector<double> ysites;  // the y-positions of all agents (required for computing voronoi partition)
		std::vector<double> xborder;  // the x-coordinates of the vertices defining the domain
		std::vector<double> yborder;  // the y-coordinates of the vertices defining the domain
		double myx;  // my current x position
		double myy;  // my current y position
		double mytheta;  // my current orientation
		std::vector<double> xvert;  // my current voronoi partition x-coordinates
		std::vector<double> yvert;  // my current voronoi partition y-coordinates

		// extract the id of the agent from the namespace string
		int extractAgentId(std::string);

		// subscriber callback for vrpn data
		void cbVrpndataUpdate(const geometry_msgs::TransformStamped::ConstPtr&, int);

		// used to publish the current agent location, orientation and voronoi partition if 'publish_data' is true
		void publishdata();

	public:

		VrpnAgentInterface(int nagents, std::vector<double> borderx, std::vector<double> bordery, double rate, bool data_publish = true)
		{
			vrpn_updated = false;
			n = nagents;
			xborder = borderx;
			yborder = bordery;
			publish_data = data_publish;
			xsites.resize(n);
			ysites.resize(n);
			vrpndata_sub.resize(n);

			std::string ns = ros::this_node::getNamespace();
			std::cout<<"The namespace is : "<<ns<<std::endl;
			size_t ind = ns.find_last_not_of("0123456789");
			myid = atoi((ns.substr(ind+1)).c_str());
			std::cout<<"Id of the agent is :"<<myid<<std::endl;

			//std::string str1 = "/tb" + str + "/voronoidata";
			voronoidata_pub = nh.advertise<turtlebotcommander::Polygonvert2d>("agentvrpndata",100);
			//std::string str2 = "/tb" + str + "/vrpndata";
			for(int i=0; i<n; i++) {
				std::string str = boost::lexical_cast<std::string>(i);  // convert number i to string
				vrpndata_sub[i] = nh.subscribe<geometry_msgs::TransformStamped> ( "/tb"+str+"/vrpndata", 1000, boost::bind(&VrpnAgentInterface::cbVrpndataUpdate, this, _1, i));
				// change the name of the subscriber topic
			}

			// timer code
			ros::Rate loop_rate(rate);
			while(ros::ok())
			{
				if(publish_data==true) {
					publishdata();
				}
	
				ros::spinOnce();
				loop_rate.sleep();
			}
		}
	
	void updateFromVrpn(const geometry_msgs::TransformStamped::ConstPtr& msg, int id)
	{
		if(vrpn_updated==false) {
				vrpn_updated = true;
		}

		xsites[id] = msg->transform.translation.x;
		ysites[id] = msg->transform.translation.y;
		if(id==myid) {
			myx = xsites[id];
			myy = ysites[id];
			mytheta= msg->transform.rotation.w;  // need to compute theta properly
		}

	}

};

void VrpnAgentInterface::cbVrpndataUpdate(const geometry_msgs::TransformStamped::ConstPtr& msg, int id)
{
	if(vrpn_updated==false) {
		vrpn_updated = true;
	}

	xsites[id] = msg->transform.translation.x;
	ysites[id] = msg->transform.translation.y;
	if(id==myid) {
		myx = xsites[id];
		myy = ysites[id];
		mytheta= msg->transform.rotation.w;  // need to compute theta properly
	}
	
}

/*
void cbVrpndataUpdate(const geometry_msgs::TransformStamped::ConstPtr& msg, int id, VrpnAgentInterface *ptr)
{
	ptr->updateFromVrpn(msg, id);
}
*/

void VrpnAgentInterface::publishdata()
{

	turtlebotcommander::Polygonvert2d msg;

	if(vrpn_updated==true) {

		for(int j=0; j<n; j++) {
			compute_voronoi(j, n, xborder, yborder, xsites, ysites, xvert, yvert);
		}

	}


	msg.myx = myx;
	msg.myy = myy;
	msg.mytheta = mytheta;
	msg.xvertices = xvert;
	msg.yvertices = yvert;
	
	// publish the data to the corresponding topic
	voronoidata_pub.publish(msg);

	vrpn_updated = false;
}
