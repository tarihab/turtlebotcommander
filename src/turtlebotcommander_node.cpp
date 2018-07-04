#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>

#include <string.h>

#include <turtlebotcommander/Polygonvert2d.h>
#include "polyintegration.h"

#define LOOP_RATE 10.0 // modify in realtime using parameter "/looprates/tbcommander"

// for simpleControl 
#define LINVEL_SETPOINT 0.2
#define ANGVEL_SETPOINT 0.8
#define LINVEL_STEPSIZE 0.01
#define ANGVEL_STEPSIZE 0.1
// the above values can be modified in real time by correspondingly changing the
// parameters linvel_setpoint, angvel_setpoint, linvel_stepsize, angvel_stepsize

// for coverageControl
#define NAGENTS 1 // modify in realtime using parameter "/domain/nagents"
#define CTRL_GAIN 1 // modify in realtime using parameter "/gains/cgain"
#define ADAPT_GAIN 1 // modify in realtime using parameter "/gains/again"

class TurtlebotCommand
{

	private:
		ros::NodeHandle nh;
		ros::Publisher cmdvel_pub;
		ros::Subscriber odom_sub, vrpn_sub;

		// id of the robot
		int32_t myid;

		// current position and velocity
		geometry_msgs::Pose2D curr_state;
		geometry_msgs::Pose2D curr_vel;
		geometry_msgs::Vector3 curr_linvel;
		geometry_msgs::Vector3 curr_angvel;
		ros::Time curr_tstamp;

		// current velocity from odometry
		geometry_msgs::Vector3 curr_linvel_odom;
		geometry_msgs::Vector3 curr_angvel_odom;

		// previous position
		// geometry_msgs::Pose2D prev_state;
		// ros::Time prev_tstamp;

		// command velocities
		geometry_msgs::Vector3 cmd_linvel;
		geometry_msgs::Vector3 cmd_angvel;

		// total no.of agents involved
		int nagents;

		// voronoi partition of the agent
		std::vector<double> xpartition;
		std::vector<double> ypartition;

		// related to phi function
		std::vector<double> centrex;
		std::vector<double> centrey;
		std::vector<double> sd;
		std::vector<double> strength;

		// gains
		int cgain;  // control gain
		int again;  // adaptation gain

		// rate at which the main loop is run.. should be ideally lower than the rate at which vrpn 
		// interface loop is run
		double looprate;

		// flag indicating whether to use vrpn data to compute velocity instead of odometry data
		bool use_vrpn_velocity;

		// flag indicating if new odometry information is available..
		bool odometry_updated;

		// flag indicating if new odometry information is available..
		bool vrpn_updated;

		// callback function for updating odometry info..
		void cbOdometryUpdate(const nav_msgs::Odometry&);

		// callback function for updating info from position tracking system (vrpn)..
		void cbVrpnUpdate(const turtlebotcommander::Polygonvert2d&);

		// compute and publish command velocity.. run with a timer..
		void simpleControlBot();
	
		double l2_norm(std::vector<double> const&);
		double l2_norm(double, double);
	
		// coverage control algorithm implementation.. run with a timer..
		void coverageControlBot();

	public:

		double phi_fcn(double, double, void*);
		double qphi_fcn(double, double, void*);

		// constructor version 1
		TurtlebotCommand(bool flag=true)
		{
			nh.param("/looprates/tbcommander", looprate, LOOP_RATE);
			vrpn_updated = false;
			odometry_updated = false;
			cmdvel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",100);
			odom_sub = nh.subscribe("odom",1000,&TurtlebotCommand::cbOdometryUpdate, this);
			vrpn_sub = nh.subscribe("agentvrpndata",1000,&TurtlebotCommand::cbVrpnUpdate, this);
			nh.param("/domain/nagents", nagents, NAGENTS);

			use_vrpn_velocity = flag;
			
			nh.getParam("/phi/centrex", centrex);
			nh.getParam("/phi/centrey", centrey);
			nh.getParam("/phi/sd", sd);
			nh.getParam("/phi/strength", strength);

			nh.param("/gains/cgain", cgain, CTRL_GAIN);
			nh.param("/gains/again", again, ADAPT_GAIN);

			// timer code
			ros::Rate loop_rate(looprate);
			while(ros::ok())
			{
				simpleControlBot();
	
				ros::spinOnce();
				loop_rate.sleep();
			}
		}

};

void TurtlebotCommand::cbOdometryUpdate(const nav_msgs::Odometry& msg)
{
	//curr_state.x = msg.pose.pose.position.x;
	//curr_state.y = msg.pose.pose.position.y;

	curr_linvel_odom.x = msg.twist.twist.linear.x;

	curr_angvel_odom.z = msg.twist.twist.angular.z;

	if(odometry_updated==false)
	{
		odometry_updated = true;
	}

	if(use_vrpn_velocity==false) {
		curr_linvel = curr_linvel_odom;
		curr_angvel = curr_angvel_odom;
	}
}

void TurtlebotCommand::cbVrpnUpdate(const turtlebotcommander::Polygonvert2d& msg)
{
	curr_state.x = msg.myx;
	curr_state.y = msg.myy;
	curr_state.theta = msg.mytheta;

	curr_vel.x = msg.myvelx;
	curr_vel.y = msg.myvely;
	curr_vel.theta = msg.myveltheta;

	curr_tstamp = msg.tstamp;

	xpartition = msg.xvertices;
	ypartition = msg.yvertices;

	if(vrpn_updated==false) {
		vrpn_updated = true;
	}

}

double TurtlebotCommand::l2_norm(std::vector<double> const& u) {
    double accum = 0.;
    for (int i = 0; i < u.size(); ++i) {
        accum += u[i] * u[i];
    }
    return sqrt(accum);
}

double TurtlebotCommand::l2_norm(double x, double y) {
    double accum = 0.;
    accum += (x*x);
    accum += (y*y);
    return sqrt(accum);
}

double TurtlebotCommand::phi_fcn(double qx, double qy, void* ptr)
{
	double phival = 0.0;
	size_t nc = centrex.size();
	
	for(int i=0; i<nc; i++) {
		phival = phival + strength[i]*(exp(-(pow(l2_norm(qx-centrex[i], qy-centrey[i]),2))/(sd[i]*sd[i])));	
	}

	return phival;
}

double TurtlebotCommand::qphi_fcn(double qx, double qy, void* ptr)
{
	int i = *(int*) ptr;
	// i=0 corresponds to qx, i=1 corresponds to qy

	double qphival = 0.0;
	
	if(i==0) {
		qphival = qx*phi_fcn(qx,qy,NULL);
	}
	else if(i==1) {
		qphival = qy*phi_fcn(qx,qy,NULL);
	}

	return qphival;
}

double phi_wrapper(void* optr, double qx, double qy, void* ptr)
{
	TurtlebotCommand* op = (TurtlebotCommand*) optr;
	double ret;
	ret = op->phi_fcn(qx, qy, ptr);
	return ret;
}

double qphi_wrapper(void* optr, double qx, double qy, void* ptr)
{
	TurtlebotCommand* op = (TurtlebotCommand*) optr;
	double ret;
	ret = op->qphi_fcn(qx, qy, ptr);
	return ret;
}

void TurtlebotCommand::simpleControlBot()
{
	if(odometry_updated==true)
	{
		double linvel_des, angvel_des, linvel_step, angvel_step;
		nh.param("linvel_setpoint", linvel_des, LINVEL_SETPOINT);
		nh.param("angvel_setpoint", angvel_des, ANGVEL_SETPOINT);
		nh.param("linvel_stepsize", linvel_step, LINVEL_STEPSIZE);
		nh.param("angvel_stepsize", angvel_step, ANGVEL_STEPSIZE);
		// put code for computing new command velocities
		cmd_linvel.x = linvel_des;
		cmd_angvel.z = angvel_des;
		/*
		if(curr_linvel.x < linvel_des)
		{
			cmd_linvel.x = curr_linvel.x + linvel_step;
		}
		else
		{
			cmd_linvel.x = curr_linvel.x - linvel_step;
		}

		if(curr_angvel.z < angvel_des)
		{
			cmd_angvel.z = curr_angvel.z + angvel_step;
		}
		else
		{
			cmd_angvel.z = curr_angvel.z - angvel_step;
		}
		*/
	}
	// if odometry data is not updated, use the old command velocities..

	// put code for publishing command velocities
	geometry_msgs::Twist cmd_velocity;	
	cmd_velocity.linear = cmd_linvel;
	cmd_velocity.angular = cmd_angvel;

	cmdvel_pub.publish(cmd_velocity);

	// clear the odometry flag for next cycle
	odometry_updated = false;

}

void TurtlebotCommand::coverageControlBot()
{
	int i;
	double Mvi, Lvi1, Lvi2, Cvi1, Cvi2;

	/**********************************************************************************************
	************ calculate the centroid of the current voronoi partition **************************
	**********************************************************************************************/
	Mvi = polyintegrate_gl(xpartition, ypartition, &phi_wrapper, this, NULL, 512);
	i = 1;
	Lvi1 = polyintegrate_gl(xpartition, ypartition, &qphi_wrapper, this, (void*) &i, 512);
	i = 2;
	Lvi2 = polyintegrate_gl(xpartition, ypartition, &qphi_wrapper, this, (void*) &i, 512);
	
	// the two components of the centroid
	Cvi1 = Lvi1/Mvi;
	Cvi2 = Lvi2/Mvi;

	/**********************************************************************************************
	******************************** compute the control ******************************************
	**********************************************************************************************/
	double e1 = curr_state.x - Cvi1;
	double e2 = curr_state.y - Cvi2;
	double th = curr_state.theta;
	double d;
	nh.param("/robot/d", d, 0.05);
	// linear velocity command
	cmd_linvel.x = -cgain*(cos(th)*e1 + sin(th)*e2);
	cmd_linvel.y = 0.0;
	cmd_linvel.z = 0.0;
	// angular velocity command
	cmd_angvel.z = -cgain*(1/d)*(-sin(th)*e1 + cos(th)*e2);
	cmd_angvel.x = 0.0;
	cmd_angvel.y = 0.0;

	/**********************************************************************************************
	******************************** publish the control values ***********************************
	**********************************************************************************************/
	geometry_msgs::Twist cmd_velocity;	
	cmd_velocity.linear = cmd_linvel;
	cmd_velocity.angular = cmd_angvel;

	cmdvel_pub.publish(cmd_velocity);

	vrpn_updated = false;
}

int main(int argc, char **argv)  // node main function
{
	ros::init(argc, argv, "turtlebot_commander");	// initializes Node Name

	//ros::NodeHandle nh;	// Node handle declaration for comm. with ROS system
	//ros::Publisher cmdvel_turtlebot_pub = nh.advertise<Twist>("cmd_vel",100);

	// the argument to the constructor is boolean indicating whether 
	// to use velocity calculated from vrpn data (else the code will 
	// use the velocity from odometry data)..
	TurtlebotCommand turtlebotCommander(true);

	return 0;
 
}
