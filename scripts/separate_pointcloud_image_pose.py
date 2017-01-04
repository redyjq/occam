#!/usr/bin/env python
import rospy
from beam_joy.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import *
import dynamic_reconfigure.client

# Author: Zach Zweig-Vinegar
# This ROS Node converts combined msg into pointcloud and img msgs

class SeparatePointcloudImagePose:
	def __init__(self):
		rospy.init_node('SeparatePointcloudImagePose')

		rospy.Subscriber('/occam/points_rgb_odom', PointcloudImagePose, self.pc_rgb_odom_callback, queue_size=1)
		self.img_pubs = []
		for i in range(5):
			self.img_pubs.append(rospy.Publisher("/occam/rgb/rgb"+str(i), Image, queue_size=1))
		self.pc_pub = rospy.Publisher("/occam/points", PointCloud2, queue_size=1)
		# self.pc_num_points_pub = rospy.Publisher("/occam/num_points", Float64, queue_size=1)
		rospy.loginfo("Initialized node.")

		# client = dynamic_reconfigure.client.Client('beam_occam')
		# params = {
		# 	'OCCAM_LEAF_SIZE' : 15,
		# 	'OCCAM_PREFERRED_BACKEND' : 1,
		# 	'OCCAM_AUTO_EXPOSURE' : True,
		# 	'OCCAM_AUTO_GAIN' : True,
		# 	'OCCAM_BM_PREFILTER_TYPE' : 1,
		# 	'OCCAM_BM_PREFILTER_SIZE' : 9,
		# 	'OCCAM_BM_PREFILTER_CAP' : 31,
		# 	'OCCAM_BM_SAD_WINDOW_SIZE' : 15,
		# 	'OCCAM_BM_MIN_DISPARITY' : 0,
		# 	'OCCAM_BM_NUM_DISPARITIES' : 64,
		# 	'OCCAM_BM_TEXTURE_THRESHOLD' : 10,
		# 	'OCCAM_BM_UNIQUENESS_RATIO' : 15,
		# 	'OCCAM_BM_SPECKLE_RANGE' : 120,
		# 	'OCCAM_BM_SPECKLE_WINDOW_SIZE' : 400,
		# 	'OCCAM_FILTER_LAMBDA' : 30,
		# 	'OCCAM_FILTER_SIGMA' : 10,
		# 	'OCCAM_FILTER_DDR' : 5
		# }
		# config = client.update_configuration(params)

		rospy.spin()

	def pc_rgb_odom_callback(self, data):
		for i in range(len(data.imgs)):
			self.img_pubs[i].publish(data.imgs[i])
		self.pc_pub.publish(data.pc)

		# self.avg_point(data.pc)
		# print "size(data.pc): %d" % (data.pc.width)
		# pc_size = data.pc.width
		# self.pc_num_points_pub.publish(float(pc_size))

	def avg_point(self, pc):
		xs, ys, zs, n = 0, 0, 0, 0
		points = pc2.read_points(pc, field_names = ("x", "y", "z", "rgba"))
		for p in points:
			# print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])
			xs += p[0]
			ys += p[1]
			zs += p[2]
			n += 1
		xs /= n
		ys /= n
		zs /= n
		print "avg: (%.3f, %.3f, %.3f)" % (xs, ys, zs)


if __name__ == '__main__':
	SeparatePointcloudImagePose()