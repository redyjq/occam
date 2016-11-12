#!/usr/bin/env python
import rospy
from beam_joy.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import *
import StringIO

# Author: Zach Zweig-Vinegar
# This ROS Node converts pointcloud and img msgs into a combined msg

class ExtractPointsStitched:
	def __init__(self):
		rospy.init_node('ExtractPointsStitched')

		odom_topic = "/beam/odom"
		image_topic = "/occam/stitched"
		pc2_topic = "/occam/points"

		rospy.Subscriber(image_topic, Image, self.img_callback, queue_size=1)
		rospy.Subscriber(pc2_topic, PointCloud2, self.pc_callback, queue_size=1)
		rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)

		self.bridge = CvBridge()

		self.odom = None		
		self.pc = None
		rospy.loginfo("Initialized node.")

		rospy.spin()

	def save_odom(self, filename, odom):
		rospy.loginfo("Save odom")
		rospy.loginfo(odom)
		with open(filename, "w") as text_file:
			odom_str = StringIO.StringIO()
			print >>odom_str, odom
			text_file.write(odom_str.getvalue())

	def img_callback(self, data):
		if(self.odom):
			# convert sensor_msgs/Image to CvMat
			try:
				cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
			except CvBridgeError as e:
				print(e)

			# get timestamp
			timestamp = data.header.stamp.to_sec()

			# save odom to text file, named with timestamp
			self.save_odom("%.3f.txt" % timestamp, self.odom)

			# save as image as png, named with timestamp
			rospy.loginfo("Save image")
			cv2.imwrite("%.3f.png" % timestamp, cv_image)

	def odom_callback(self, data):
		self.odom = data

	def pc_callback(self, data):
		self.pc = data

if __name__ == '__main__':
	ExtractPointsStitched()