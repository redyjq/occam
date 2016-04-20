import numpy as np

# kinect already in meters, occam in centimeters
lines = open("occam/pointcloud0.pcd", "rb").readlines()[11:]
#lines = open("pointclouds_kinect/pointcloud444.pcd", "rb").readlines()[11:]
points = [line.strip().split(" ")[:-1] for line in lines]
points = [[float(i)/100 for i in p] for p in points if p[0] != 'nan']

minx = min(points, key=lambda point: point[0])[0]
miny = min(points, key=lambda point: point[1])[1]
minz = min(points, key=lambda point: point[2])[2]

maxx = max(points, key=lambda point: point[0])[0]
maxy = max(points, key=lambda point: point[1])[1]
maxz = max(points, key=lambda point: point[2])[2]


import pdb; pdb.set_trace()
vol = (maxx - minx) * (maxy - miny) * (maxz - minz)
print "volume: ", vol
print "points per m^3: ", float(len(points)) / vol

#count = 0
#for point in points:
#    in_x = point[0] > 0 and point[0] < 1
#    in_y = point[1] > 0 and point[1] < 1
#    in_z = point[2] > 0 and point[2] < 1
#
#    if in_x and in_y and in_z:
#        count += 1
#print "points per m^2: ", count

# results:
# kinect point clouds: 7202 points/m^3
# occam point clouds :  253 points/m^3

# voxel grid w leaf size 0.02 -> all points within a 2x2x2cm box become 1 point
# how many points are in a 2x2x2cm box of density 7202 points/m^3?
# 8cm^3 -> 0.000008m^3

# 125000 2x2x2 cubes in a m^3
