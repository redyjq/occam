#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import time
from tf.transformations import *
from math import pi, sin, cos

# Author: Zach Zweig-Vinegar
# This ROS Node publishes fake odometry msgs for the beam robot

class OdomPub:
	def __init__(self):
		rospy.init_node('FakeOdomPub')
		# odom tester
		self.odom_pub = rospy.Publisher('/beam/odom', Odometry, queue_size=1)

		# publishes fake odometry at 10 Hz
		rate = rospy.Rate(10)
		max_update_time = 3
		odom = Odometry()
		odom.child_frame_id = "base"
		odom.header.frame_id = "odom"
		t = 0
		step = 2*pi/360
		while not rospy.is_shutdown():
			odom.header.stamp = rospy.Time.now()

			# Set fake odom data for testing
			t += step
			if t > 2*pi:
				t = 0
			odom.pose.pose.position.x = cos(t)
			odom.pose.pose.position.y = sin(t)

			# odom.pose.pose.orientation.x = .707
			# odom.pose.pose.orientation.z = .707
			odom.pose.pose.orientation.w = 1

			self.odom_pub.publish(odom)
			rospy.loginfo("Published fake odom")
			# rospy.loginfo(odom)
			rate.sleep()

		rospy.spin()

if __name__ == '__main__':
	OdomPub()