#include <fstream>
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

#include <rgbsensor/rgbdata.h>

#define LOOP_RATE 20.0 // modify in realtime using parameter "/looprates/tbcommander"

// for simpleControl 
#define LINVEL_SETPOINT 0.2
#define ANGVEL_SETPOINT 0.8
#define LINVEL_STEPSIZE 0.01
#define ANGVEL_STEPSIZE 0.1
// the above values can be modified in real time by correspondingly changing the
// parameters linvel_setpoint, angvel_setpoint, linvel_stepsize, angvel_stepsize

// for coverageControl
#define NAGENTS 1 // modify in realtime using parameter "/domain/nagents"
#define CTRL_GAIN 1.0 // modify in realtime using parameter "/gains/cgain"
#define ADAPT_GAIN 1.0 // modify in realtime using parameter "/gains/again"
#define PARAM_INIT_VALUE 0.1

struct rgb_struct
{
	int r;
	int g;
	int b;
	int c;
	long colortemp;
	int lux;
};

class TurtlebotCommand
{

	private:
		ros::NodeHandle nh;
		ros::Publisher cmdvel_pub;
		ros::Subscriber odom_sub, vrpn_sub, rgb_sub;

		// id of the robot
		int32_t myid;

		// current position and velocity
		geometry_msgs::Pose2D curr_state;
		geometry_msgs::Pose2D curr_vel;
		geometry_msgs::Vector3 curr_linvel;
		geometry_msgs::Vector3 curr_angvel;
		ros::Time curr_tstamp;
		double Cvix, Cviy; // current centroid coordinates

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

		int np; // number of parameters
		std::vector<double> ahat; // parameter estimate
		std::vector<double> ahat_prev; // parameter estimate previous value
		std::vector<std::vector<double> > Lambda; // filter variable 1
		std::vector<std::vector<double> > Lambda_prev; // filter variable 1 previous value
		std::vector<double> lambda; // filter variable 2
		std::vector<double> lambda_prev; // filter variable 2 previous value
		ros::Time tstamp;
		ros::Time tstamp_prev;

		// gains
		double cgain;  // control gain
		double again;  // adaptation gain

		// rate at which the main loop is run.. should be ideally lower than the rate at which vrpn 
		// interface loop is run
		double looprate;

		// flag indicating whether to use vrpn data to compute velocity instead of odometry data
		bool use_vrpn_velocity;

		// flag indicating if new odometry information is available..
		bool odometry_updated;

		// flag indicating if new odometry information is available..
		bool vrpn_updated;

		// flag indicating if the algorithm is adaptive or not
		int adaptive;

		// rgb measurement
		struct rgb_struct rgb_feedback;
		// flag indicating if new rgb data is available..
		bool rgb_updated;
		// flag indicating whether to use sensor values or not..
		int usesensors;

		int bias_on;

		// file for logging the data
		std::ofstream filelog;

		// flag indicating whether to log data or not..
		int logdata;
		double lograte; // as a fraction of the looprate..

		// callback function for updating odometry info..
		void cbOdometryUpdate(const nav_msgs::Odometry&);

		// callback function for updating info from position tracking system (vrpn)..
		void cbVrpnUpdate(const turtlebotcommander::Polygonvert2d&);

		// callback function for rgb sensor data..
		void cbRgbUpdate(const rgbsensor::rgbdata&);

		// compute and publish command velocity.. run with a timer..
		void simpleControlBot();
	
		double l2_norm(std::vector<double> const&);
		double l2_norm(double, double);
		double dotproduct(std::vector<double> const&, std::vector<double> const&);
	
		// coverage control algorithm implementation.. run with a timer..
		void coverageControlBot(int);

	public:

		double phi_fcn(double, double, void*);
		double qphi_fcn(double, double, void*);
		double adaptation_integrand(double, double, void*);

		// constructor version 1
		TurtlebotCommand(bool flag=true)
		{
			nh.param("/looprates/tbcommander", looprate, LOOP_RATE);
			//nh.getParam("/looprates/tbcommander", looprate);
			std::cout<<"Loop rate set at "<<looprate<<" Hz"<<std::endl;
			vrpn_updated = false;
			odometry_updated = false;
			rgb_updated = false;
			cmdvel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",100);
			odom_sub = nh.subscribe("odom",1000,&TurtlebotCommand::cbOdometryUpdate, this);
			vrpn_sub = nh.subscribe("agentvrpndata",1000,&TurtlebotCommand::cbVrpnUpdate, this);
			rgb_sub = nh.subscribe("rgbcdata",1000,&TurtlebotCommand::cbRgbUpdate, this);
			nh.param("/domain/nagents", nagents, NAGENTS);

			use_vrpn_velocity = flag;
			
			nh.param("/phi/bias_on", bias_on, 0);

			nh.getParam("/phi/centrex", centrex);
			nh.getParam("/phi/centrey", centrey);
			nh.getParam("/phi/sd", sd);
			nh.getParam("/phi/strength", strength);
			np = static_cast<int>(centrex.size());
			double paramInitVal;
			nh.param("/adaptation/paramInitValue", paramInitVal, PARAM_INIT_VALUE);
			ahat.resize(np,paramInitVal);
			ahat_prev.resize(np,paramInitVal);
			Lambda.resize(np, std::vector<double>(np,0.0));
			Lambda_prev.resize(np, std::vector<double>(np,0.0));
			lambda.resize(np,0.0);
			lambda_prev.resize(np,0.0);
			std::cout<<"centrex: "<<centrex[0]<<std::endl;
			std::cout<<"centrey: "<<centrey[0]<<std::endl;
			std::cout<<"sd: "<<sd[0]<<std::endl;
			std::cout<<"strength: "<<strength[0]<<std::endl;

			// xpartition and ypartition initialized to the border vertices
			nh.getParam("/domain/borderx", xpartition);
			nh.getParam("/domain/bordery", ypartition);

			nh.param("/gains/cgain", cgain, CTRL_GAIN);
			nh.param("/gains/again", again, ADAPT_GAIN);
			std::cout<<"Controller gain: "<<cgain<<std::endl;
			std::cout<<"Adaptation gain: "<<again<<std::endl;

			cmd_linvel.x = 0.0;
			cmd_linvel.y = 0.0;
			cmd_linvel.z = 0.0;
			cmd_angvel.x = 0.0;
			cmd_angvel.y = 0.0;
			cmd_angvel.z = 0.0;
			curr_state.x = 0.0;
			curr_state.y = 0.0;
			curr_state.theta = 0.0;
			curr_vel.x = 0.0;
			curr_vel.y = 0.0;
			curr_vel.theta = 0.0;

			std::string ns = ros::this_node::getNamespace();
			std::cout<<"The namespace is : "<<ns<<std::endl;
			size_t ind = ns.find_last_not_of("0123456789");
			myid = atoi((ns.substr(ind+1)).c_str());
			std::cout<<"Id of the agent is :"<<myid<<std::endl;

			nh.param("/adaptive", adaptive, 0);
			std::cout<<"Adaptive :"<<adaptive<<std::endl;

			nh.param("/usesensors", usesensors, 0);
			std::cout<<"Use sensors :"<<usesensors<<std::endl;

			nh.param("/logdata", logdata, 0);
			std::cout<<"Data logging :"<<logdata<<std::endl;
			nh.param("/datalog_rate", lograte, 0.0);
			std::cout<<"Data logging rate :"<<lograte<<std::endl;
			double logflag = 0.0;

			// convert number myid to string
			std::string id_str = boost::lexical_cast<std::string>(myid);  
			std::string str = "/home/rihab/agent" + id_str + "_" + "log";
			filelog.open(str.c_str(), std::ios::out | std::ios::trunc);


			tstamp_prev = ros::Time::now();

			int stopagents;
			int stopflag = 1;
			// timer code
			ros::Rate loop_rate(looprate);
			while(ros::ok())
			{
				nh.param("/stopAgents", stopagents, 0);
				if(stopagents==0) {
					stopflag = 1;
					//simpleControlBot();
					coverageControlBot(adaptive);
				}
				else { // stop the agents
					if(stopflag==1) {
						std::cout<<"Agent stopped"<<std::endl;
						stopflag = 0;
					}
					geometry_msgs::Twist cmd_velocity;	
					cmd_linvel.x = 0.0;
					cmd_angvel.z = 0.0;
					cmd_velocity.linear = cmd_linvel;
					cmd_velocity.angular = cmd_angvel;
					cmdvel_pub.publish(cmd_velocity);
				}

				logflag = logflag + lograte;	
				if(logflag >= 1.0) {
					logflag = 0.0;
					if(logdata>0) {
						filelog<<curr_tstamp.toSec()<<"\t"<<curr_state.x<<"\t"<<curr_state.y<<"\t"<<curr_state.theta<<"\t"<<cmd_linvel.x<<"\t"<<cmd_angvel.z<<"\t"<<Cvix<<"\t"<<Cviy<<"\t"<<rgb_feedback.r<<"\n";
					}
				}
	
				ros::spinOnce();
				loop_rate.sleep();
			}

			filelog.close();
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
	
	// std::cout<<"Updated VRPN data"<<std::endl;

}

void TurtlebotCommand::cbRgbUpdate(const rgbsensor::rgbdata& msg)
{
	rgb_feedback.r = msg.r;
	rgb_feedback.g = msg.g;
	rgb_feedback.b = msg.b;
	rgb_feedback.c = msg.c;
	rgb_feedback.colortemp = msg.temp;
	rgb_feedback.lux = msg.lum;

	if(rgb_updated==false) {
		rgb_updated = true;
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
		
double TurtlebotCommand::dotproduct(std::vector<double> const& v1, std::vector<double> const& v2) {
	double dotp = 0.0;
	int n = static_cast<int>(v1.size());
	for(int i=0; i<n; i++) {
		dotp += v1[i]*v2[i];
	}
	return dotp;
}

double TurtlebotCommand::phi_fcn(double qx, double qy, void* ptr)
{
	double phival = 0.0;
	size_t nc = centrex.size();
	int flag = 0;
	if(ptr==NULL && adaptive==1) {
		flag = 1;	
	}
	
	if(flag==0) {  // not adaptive control
		for(int i=0; i<nc; i++) {
			phival = phival + strength[i]*(exp(-(pow(l2_norm(qx-centrex[i], qy-centrey[i]),2))/(sd[i]*sd[i])));	
		}
	} else { // adaptive control case
		for(int i=0; i<nc; i++) {
			phival = phival + ahat[i]*(exp(-(pow(l2_norm(qx-centrex[i], qy-centrey[i]),2))/(sd[i]*sd[i])));	
		}
	}

	if(bias_on > 0) {
		phival = phival + strength[nc]*1.0;
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

double TurtlebotCommand::adaptation_integrand(double qx, double qy, void* ptr)
{
	int i = *(int*) ptr;

	double val = 0.0;
	double Ki = exp(-(pow(l2_norm(qx-centrex[i], qy-centrey[i]),2))/(sd[i]*sd[i]));
	double e1 = curr_state.x - Cvix;
	double e2 = curr_state.y - Cviy;
	/* double tmp1 = (qx-curr_state.x)*e1;
	double tmp2 = (qy-curr_state.y)*e2;
	val = Ki*(tmp1+tmp2); */
	val = Ki*(((qx-curr_state.x)*e1)+((qy-curr_state.y)*e2));
	
	return val;
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

double adaptation_int_wrapper(void* optr, double qx, double qy, void* ptr)
{
	TurtlebotCommand* op = (TurtlebotCommand*) optr;
	double ret;
	ret = op->adaptation_integrand(qx, qy, ptr);
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

void TurtlebotCommand::coverageControlBot(int adaptive)
{
	int i;
	double Mvi, Lvi1, Lvi2;
	
	if(adaptive==0) {
		/**********************************************************************************************
		************ calculate the centroid of the current voronoi partition **************************
		**********************************************************************************************/
		if(vrpn_updated==true) {

			// this is just for testing.. comment these when running 
			// nh.getParam("/domain/borderx", xpartition);
			// nh.getParam("/domain/bordery", ypartition);
			// curr_state.x = 0.0;
			// curr_state.y = 0.0;
			// curr_state.theta = 0.0;
			///////////////////////////////////////////////////// //

			Mvi = polyintegrate_gl(xpartition, ypartition, &phi_wrapper, this, NULL, 512);
			i = 0;
			Lvi1 = polyintegrate_gl(xpartition, ypartition, &qphi_wrapper, this, (void*) &i, 512);
			i = 1;
			Lvi2 = polyintegrate_gl(xpartition, ypartition, &qphi_wrapper, this, (void*) &i, 512);
	
			// the two components of the centroid
			Cvix = Lvi1/Mvi;
			Cviy = Lvi2/Mvi;

		/**********************************************************************************************
		******************************** compute the control ******************************************
		**********************************************************************************************/
			double e1 = curr_state.x - Cvix;
			double e2 = curr_state.y - Cviy;
			double th = curr_state.theta;
			double d;
			nh.param("/robot/d", d, 0.05);

			// linear velocity command
			cmd_linvel.x = -cgain*(cos(th)*e1 + sin(th)*e2);

			// angular velocity command
			// cmd_angvel.z = -cgain*(1/d)*(-sin(th)*e1 + cos(th)*e2);
			cmd_angvel.z = -cgain*0.05*(1/d)*(-sin(th)*e1 + cos(th)*e2);
		}
	}
	else {	// adaptive control algorithm

		/**********************************************************************************************
		************ calculate the centroid of the current voronoi partition **************************
		**********************************************************************************************/
		if(vrpn_updated==true) {

			// this is just for testing.. comment these when running 
			// nh.getParam("/domain/borderx", xpartition);
			// nh.getParam("/domain/bordery", ypartition);
			// curr_state.x = 0.0;
			// curr_state.y = 0.0;
			// curr_state.theta = 0.0;
			///////////////////////////////////////////////////// //

			Mvi = polyintegrate_gl(xpartition, ypartition, &phi_wrapper, this, NULL, 512);
			i = 0;
			Lvi1 = polyintegrate_gl(xpartition, ypartition, &qphi_wrapper, this, (void*) &i, 512);
			i = 1;
			Lvi2 = polyintegrate_gl(xpartition, ypartition, &qphi_wrapper, this, (void*) &i, 512);
	
			// the two components of the centroid
			Cvix = Lvi1/Mvi;
			Cviy = Lvi2/Mvi;

		/**********************************************************************************************
		******************************** compute the control ******************************************
		**********************************************************************************************/
			double e1 = curr_state.x - Cvix;
			double e2 = curr_state.y - Cviy;
			double th = curr_state.theta;
			double d;
			nh.param("/robot/d", d, 0.05);

			// linear velocity command
			cmd_linvel.x = -cgain*(cos(th)*e1 + sin(th)*e2);

			// angular velocity command
			// cmd_angvel.z = -cgain*(1/d)*(-sin(th)*e1 + cos(th)*e2);
			cmd_angvel.z = -cgain*0.05*(1/d)*(-sin(th)*e1 + cos(th)*e2);

		/**********************************************************************************************
		******************************** adaptation law ***********************************************
		**********************************************************************************************/
			double phim;
			if(usesensors==0) {
				int i=10;
				phim = phi_fcn(curr_state.x, curr_state.y, (void *) &i); // measurement of phi
			} else {
				phim = static_cast<double>(rgb_feedback.r);	
			}

			tstamp = ros::Time::now();
			double dt = tstamp.toSec() - tstamp_prev.toSec();

			ahat_prev = ahat;
			std::vector<double> bi(np, 0.0);
			double amin, k1; 
			nh.getParam("/adaptation/paramInitValue", amin);
			nh.getParam("/adaptation/gain1", k1);
			for(int i=0; i<np; i++) {
				bi[i] = polyintegrate_gl(xpartition, ypartition, &adaptation_int_wrapper, this, (void*) &i, 512);
				bi[i] = -cgain*bi[i] - k1*(dotproduct(Lambda[i],ahat) - lambda[i]);
				if((ahat[i]==amin && bi[i]<0) || (ahat[i]<amin)) {
					bi[i] = 0.0;
				}

				ahat[i] = ahat_prev[i] + again*bi[i]*dt;
			}

			// update the filter variables
			Lambda_prev = Lambda;
			lambda_prev = lambda;
			double alpha, Ki, Kj;
			nh.getParam("/adaptation/alpha",alpha);
			for(int i=0; i<np; i++) {
				Ki = (exp(-(pow(l2_norm(curr_state.x-centrex[i], curr_state.y-centrey[i]),2))/(sd[i]*sd[i])));
				for(int j=0; j<np; j++) {
					Kj = (exp(-(pow(l2_norm(curr_state.x-centrex[j], curr_state.y-centrey[j]),2))/(sd[j]*sd[j])));
					Lambda[i][j] = exp(-alpha*dt)*Lambda_prev[i][j] + (1-exp(-alpha*dt))*Ki*Kj;
				}
				lambda[i] = exp(-alpha*dt)*lambda_prev[i] + (1-exp(-alpha*dt))*Ki*phim;
			}

			tstamp_prev = tstamp;
	
		}

		rgb_updated = false;

	}

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

	std::cout<<"Node name : "<<ros::this_node::getName()<<std::endl;
	//ros::NodeHandle nh;	// Node handle declaration for comm. with ROS system
	//ros::Publisher cmdvel_turtlebot_pub = nh.advertise<Twist>("cmd_vel",100);

	// the argument to the constructor is boolean indicating whether 
	// to use velocity calculated from vrpn data (else the code will 
	// use the velocity from odometry data)..
	TurtlebotCommand turtlebotCommander(true);

	return 0;
 
}
