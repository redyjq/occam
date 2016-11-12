#!/usr/bin/env python
import rospy
from beam_joy.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from threading import Lock

# Author: Zach Zweig-Vinegar
# This ROS Node converts pointcloud and img msgs into a combined msg

class CombinePointsStitched:
	def __init__(self):
		rospy.init_node('CombinePointsStitched')

		self.pub = rospy.Publisher('/occam/points_rgb_odom', PointcloudImagePose, queue_size=1)
		rospy.Subscriber("/occam/stitched", Image, self.img_callback, queue_size=1)
		rospy.Subscriber("/occam/points", PointCloud2, self.pc_callback, queue_size=1)

		self.img_lock = Lock()
		self.pc_lock = Lock()

		self.msg = PointcloudImagePose()
		rospy.loginfo("Initialized node.")

		rospy.spin()

	def img_callback(self, data):
		self.msg = PointcloudImagePose()
		self.msg.img = data
		rospy.loginfo("set img")
		self.pc_lock.acquire()
		rospy.loginfo("waiting for pc")

		self.pc_lock.acquire()
		rospy.loginfo("done waiting")
		self.pub.publish(self.msg)
		rospy.loginfo("published")
		self.msg = PointcloudImagePose()


	def pc_callback(self, data):
		self.msg.pc = data
		rospy.loginfo("set pc")
		self.pc_lock.release()


if __name__ == '__main__':
	CombinePointsStitched()