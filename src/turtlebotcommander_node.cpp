#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>

#include <string.h>

#define LINVEL_SETPOINT 0.25
#define ANGVEL_SETPOINT 1.0
#define LINVEL_STEPSIZE 0.01
#define ANGVEL_STEPSIZE 0.1
// the above values can be modified in real time by correspondingly changing the
// parameters linvel_setpoint, angvel_setpoint, linvel_stepsize, angvel_stepsize

class TurtlebotCommand
{

	private:
		ros::NodeHandle nh;
		ros::Publisher cmdvel_pub;
		ros::Subscriber odom_sub;

		// current position and velocity
		geometry_msgs::Vector3 curr_pos;
		geometry_msgs::Vector3 curr_linvel;
		geometry_msgs::Vector3 curr_angvel;

		// command velocities
		geometry_msgs::Vector3 cmd_linvel;
		geometry_msgs::Vector3 cmd_angvel;

		// flag indicating if new odometry information is available..
		bool odometry_updated;

		// callback function for updating odometry info..
		void cbOdometryUpdate(const nav_msgs::Odometry&);

		// compute and publish command velocity.. run with a timer..
		void controlBot();

	public:
		// constructor
		TurtlebotCommand(double rate)
		{
			odometry_updated = false;
			cmdvel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",100);
			odom_sub = nh.subscribe("odom",1000,&TurtlebotCommand::cbOdometryUpdate, this);
			
			// timer code
			ros::Rate loop_rate(rate);
			while(ros::ok())
			{
				controlBot();
	
				ros::spinOnce();
				loop_rate.sleep();
			}
		}

};

void TurtlebotCommand::cbOdometryUpdate(const nav_msgs::Odometry& msg)
{
	if(odometry_updated==false)
	{
		odometry_updated = true;
	}

	curr_pos.x = msg.pose.pose.position.x;
	curr_pos.y = msg.pose.pose.position.y;

	curr_linvel.x = msg.twist.twist.linear.x;

	curr_angvel.z = msg.twist.twist.angular.z;
}

void TurtlebotCommand::controlBot()
{
	if(odometry_updated==true)
	{
		double linvel_des, angvel_des, linvel_step, angvel_step;
		nh.param("linvel_setpoint", linvel_des, LINVEL_SETPOINT);
		nh.param("angvel_setpoint", angvel_des, ANGVEL_SETPOINT);
		nh.param("linvel_stepsize", linvel_step, LINVEL_STEPSIZE);
		nh.param("angvel_stepsize", angvel_step, ANGVEL_STEPSIZE);
		// put code for computing new command velocities
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

int main(int argc, char **argv)  // node main function
{
	ros::init(argc, argv, "turtlebot_commander");	// initializes Node Name

	//ros::NodeHandle nh;	// Node handle declaration for comm. with ROS system
	//ros::Publisher cmdvel_turtlebot_pub = nh.advertise<Twist>("cmd_vel",100);

	// the argument to the constructor is the rate of control update in Hz..
	TurtlebotCommand turtlebotCommander(10);

	return 0;
 
}
