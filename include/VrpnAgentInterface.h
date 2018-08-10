#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
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
#define NAGENTS 1
#define LOOP_RATE 20.0

class VrpnAgentInterface
{

	private:
		ros::NodeHandle nh;
		ros::Publisher voronoidata_pub;
		std::vector<ros::Subscriber> vrpndata_sub;		 	

		int32_t myid;  // the id of the agent
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

		double avgvelx;
		double avgvely;
		double avgveltheta;
		int readIndex;
		double totalx;
		double totaly;
		double totaltheta;
		double velx[5];
		double vely[5];
		double veltheta[5];

		ros::Time tstamp;
		std::vector<double> xvert;  // my current voronoi partition x-coordinates
		std::vector<double> yvert;  // my current voronoi partition y-coordinates

		// previous values
		double prevx;
		double prevy;
		double prevtheta;
		ros::Time prev_tstamp;

		double dt;
		double looprate;

		// extract the id of the agent from the namespace string
		int extractAgentId(std::string);

		// subscriber callback for vrpn data
		void cbVrpndataUpdate(const geometry_msgs::PoseStamped::ConstPtr&, int);

		// used to publish the current agent location, orientation and voronoi partition if 'publish_data' is true
		void publishdata();

	public:

		VrpnAgentInterface(int nagents, std::vector<double> borderx, std::vector<double> bordery, double rate, bool data_publish = true)
		{
			readIndex = 0;
			totalx = 0.0;
			totaly = 0.0;
			totaltheta = 0.0;

			vrpn_updated = false;
			n = nagents;
			xborder = borderx;
			yborder = bordery;
			publish_data = data_publish;
			xsites.resize(n);
			ysites.resize(n);
			vrpndata_sub.resize(n);

			looprate = rate;
			std::cout<<"Loop rate set at "<<looprate<<" Hz"<<std::endl;

			prevx = 0;
			prevy = 0;
			prevtheta = 0;
			prev_tstamp = ros::Time(0.0);

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
				vrpndata_sub[i] = nh.subscribe<geometry_msgs::PoseStamped> ( "/vrpn_client_node/tb"+str+"/pose", 1000, boost::bind(&VrpnAgentInterface::cbVrpndataUpdate, this, _1, i));
				// change the name of the subscriber topic
			}

			// timer code
			ros::Rate loop_rate(looprate);
			while(ros::ok())
			{
				if(publish_data==true) {
					publishdata();
				}
	
				ros::spinOnce();
				loop_rate.sleep();
			}
		}

		VrpnAgentInterface(bool data_publish = true)
		{
			readIndex = 0;
			totalx = 0.0;
			totaly = 0.0;
			totaltheta = 0.0;

			std::string ns = ros::this_node::getNamespace();
			std::cout<<"The namespace is : "<<ns<<std::endl;
			size_t ind = ns.find_last_not_of("0123456789");
			myid = atoi((ns.substr(ind+1)).c_str());
			std::cout<<"Id of the agent is :"<<myid<<std::endl;

			// retrieve parameters
			nh.param("/domain/nagents", n, NAGENTS);
			nh.param("/looprates/vrpninterface", looprate, LOOP_RATE);
			nh.getParam("/domain/borderx", xborder);
			nh.getParam("/domain/bordery", yborder);
			std::cout<<"Loop rate set at "<<looprate<<" Hz"<<std::endl;

			vrpn_updated = false;
			publish_data = data_publish;
			xsites.resize(n);
			ysites.resize(n);
			vrpndata_sub.resize(n);

			prevx = 0;
			prevy = 0;
			prevtheta = 0;
			prev_tstamp = ros::Time(0.0);

			//std::string str1 = "/tb" + str + "/voronoidata";
			voronoidata_pub = nh.advertise<turtlebotcommander::Polygonvert2d>("agentvrpndata",100);
			//std::string str2 = "/tb" + str + "/vrpndata";
			for(int i=0; i<n; i++) {
				std::string str = boost::lexical_cast<std::string>(i);  // convert number i to string
				vrpndata_sub[i] = nh.subscribe<geometry_msgs::PoseStamped> ( "/vrpn_client_node/tb"+str+"/pose", 1000, boost::bind(&VrpnAgentInterface::cbVrpndataUpdate, this, _1, i));
				// change the name of the subscriber topic
			}

			// timer code
			ros::Rate loop_rate(looprate);
			while(ros::ok())
			{
				if(publish_data==true) {
					publishdata();
				}
	
				ros::spinOnce();
				loop_rate.sleep();
			}
		}

	/*
	void updateFromVrpn(const geometry_msgs::PoseStamped::ConstPtr& msg, int id)
	{
		if(vrpn_updated==false) {
				vrpn_updated = true;
		}

		xsites[id] = msg->pose.position.x;
		ysites[id] = msg->pose.position.y;
		if(id==myid) {
			myx = xsites[id];
			myy = ysites[id];
			mytheta= msg->pose.orientation.w;  // need to compute theta properly
		}

	}
	*/

};

void VrpnAgentInterface::cbVrpndataUpdate(const geometry_msgs::PoseStamped::ConstPtr& msg, int id)
{
	if(id==myid) {

		xsites[id] = msg->pose.position.x;
		ysites[id] = msg->pose.position.y;

		myx = xsites[id];
		myy = ysites[id];
		tstamp = msg->header.stamp;

		dt = tstamp.toSec() - prev_tstamp.toSec();

		/**********************************************************************************
		 **************** Computing orientation theta from quaternion data ****************
		 *********************************************************************************/
		tf::Quaternion q(
		msg->pose.orientation.x,
		msg->pose.orientation.y,
		msg->pose.orientation.z,
		msg->pose.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		mytheta= yaw;

		/**********************************************************************************
		 ******* Computing velocities from position data (code not verified) **************
		 *********************************************************************************/
		totalx = totalx - velx[readIndex];
		totaly = totaly - vely[readIndex];
		totaltheta = totaltheta - veltheta[readIndex];
		
		velx[readIndex] = (myx - prevx)/dt;
		vely[readIndex] = (myy - prevy)/dt;
		veltheta[readIndex] = (mytheta - prevtheta)/dt;

		totalx = totalx + velx[readIndex];
		totaly = totaly + vely[readIndex];
		totaltheta = totaltheta + veltheta[readIndex];

		readIndex = readIndex + 1;

		if(readIndex>=5) {
			readIndex = 0;
		}

		avgvelx = totalx/5;
		avgvely = totaly/5;
		avgveltheta = totaltheta/5;

		// updating the previous variables
		prevx = myx;
		prevy = myy;
		prevtheta = mytheta;
		prev_tstamp = tstamp;
	}
	else {
		xsites[id] = msg->pose.position.x;
		ysites[id] = msg->pose.position.y;
	}

	if(vrpn_updated==false) {
		vrpn_updated = true;
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
		compute_voronoi(myid, n, xborder, yborder, xsites, ysites, xvert, yvert);
	}


	msg.myid = myid;
	msg.myx = myx;
	msg.myy = myy;
	msg.mytheta = mytheta;
	msg.myvelx = avgvelx;
	msg.myvely = avgvely;
	msg.myveltheta = avgveltheta;
	msg.tstamp = tstamp;
	msg.xvertices = xvert;
	msg.yvertices = yvert;
	
	// publish the data to the corresponding topic
	voronoidata_pub.publish(msg);

	vrpn_updated = false;
}
