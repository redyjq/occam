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
# This ROS Node extracts png, and txt files from a beam_joy/PointcloudImagePose msg

class ExtractPointcloudImagePose:
	def __init__(self):
		rospy.init_node('ExtractPointcloudImagePose')
		topic = "/occam/points_rgb_odom"
		rospy.Subscriber(topic, PointcloudImagePose, self.callback, queue_size=5)
		self.bridge = CvBridge()
		rospy.loginfo("Initialized node.")
		rospy.spin()

	def callback(self, data):
		timestamp = data.pc.header.stamp.to_sec()

		self.save_pc(data.pc)

		self.save_odom("%.3f.txt" % timestamp, data.odom)
		
		for i, img in enumerate(data.imgs):
			self.save_img("%.3f_%d.png" % (timestamp, i), img)

	def save_pc(self, pc):
		pass

	def save_odom(self, filename, odom):
		# save odom to text file, named with timestamp
		rospy.loginfo("Save odom")
		with open(filename, "w") as text_file:
			odom_str = StringIO.StringIO()
			print >>odom_str, odom
			text_file.write(odom_str.getvalue())

	def save_img(self, filename, img):
		# convert sensor_msgs/Image to CvMat
		try:
			cv_image = self.bridge.imgmsg_to_cv2(img, "passthrough")
		except CvBridgeError as e:
			print(e)

		# save as image as png, named with timestamp
		rospy.loginfo("Save image")
		cv2.imwrite(filename, cv_image)

if __name__ == '__main__':
	ExtractPointcloudImagePose()