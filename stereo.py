import cv2
import numpy as np
from matplotlib import pyplot as plt
import os
import glob
import random

"""
Code is from http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_calib3d/py_epipolar_geometry/py_epipolar_geometry.html
"""
root_dir = 'extrinsics_attempt'


def calibrate_stereo_pair(sensorpair):
    images = glob.glob('%s/*.jpg' % root_dir)
    img = cv2.imread(images[int(random.random() * len(images))], 0)
    height = 485/2
    width = 1900 / 5
    # The Occam is upright so we transpose here to orient things left-right
    # Not sure if necessary
    img1 = img[0:height, width*sensorpair:width*(sensorpair + 1)].T
    img2 = img[height:485, width*sensorpair:width*(sensorpair + 1)].T

    #img1 = cv2.imread('%s/%s/left.jpg',0)  #queryimage # left image
    #img2 = cv2.imread('%s/%s/right.jpg',0) #trainimage # right image
    #cv2.imshow('left image', img1)
    #cv2.imshow('right image', img2)
    #cv2.waitKey(0)

    sift = cv2.SIFT()

    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(img1,None)
    kp2, des2 = sift.detectAndCompute(img2,None)

    # FLANN parameters
    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks=50)

    flann = cv2.FlannBasedMatcher(index_params,search_params)
    matches = flann.knnMatch(des1,des2,k=2)

    good = []
    pts1 = []
    pts2 = []

    # ratio test as per Lowe's paper
    for i,(m,n) in enumerate(matches):
        if m.distance < 0.8*n.distance:
            good.append(m)
            pts2.append(kp2[m.trainIdx].pt)
            pts1.append(kp1[m.queryIdx].pt)

    pts1 = np.array(pts1)
    pts2 = np.array(pts2)
    F, mask = cv2.findFundamentalMat(pts1,pts2,cv2.FM_LMEDS)

    output = '%s/sensorpair%s' % (root_dir, sensorpair)
    if 'sensorpair%s' % sensorpair not in os.listdir(root_dir):
        os.mkdir(output)
    np.save(open('%s/fundamental.numpy' % output, 'w'), F)
    np.save(open('%s/mask.numpy' % output, 'w'), mask)

calibrate_stereo_pair(0)
