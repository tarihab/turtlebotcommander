#!/usr/bin/env python

import rospy
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

rospy.init_node('turtlebotcommander')

def odometryObserver(odomData):
	# fill what to do with odometry data
	# print the position and velocity every
	# 10 seconds
	pos = Vector3()
	linvel = Vector3()
	angvel = Vector3()

	pos.x = odomData.pose.pose.position.x
	pos.y = odomData.pose.pose.position.y
	pos.z = odomData.pose.pose.position.z
	linvel.x = odomData.twist.twist.linear.x
	linvel.y = odomData.twist.twist.linear.y
	linvel.z = odomData.twist.twist.linear.z
	angvel.x = odomData.twist.twist.angular.x
	angvel.y = odomData.twist.twist.angular.y
	angvel.z = odomData.twist.twist.angular.z

	print ("position:" + str(pos.x) + "," + str(pos.y) + "," + str(pos.z))
	print ("linear velocity:" + str(linvel.x) + "," + str(linvel.y) + "," + str(linvel.z))
	print ("angular velocity:" + str(angvel.x) + "," + str(angvel.y) + "," + str(angvel.z))
	# time.sleep(5)
	cmd_vel = Twist()
	if linvel.x < 0.2:
		cmd_vel.linear.x = linvel.x + 0.05
	else:
		cmd_vel.linear.x = linvel.x - 0.05

	if angvel.z < 0.2:
		cmd_vel.angular.z = angvel.z + 0.05
	else:
		cmd_vel.angular.z = angvel.z - 0.05

	cmdPub.publish(cmd_vel)
	# time.sleep(5)
	

odomSub = rospy.Subscriber('odom', Odometry, odometryObserver);
cmdPub = rospy.Publisher('cmd_vel',Twist)

rospy.spin()
