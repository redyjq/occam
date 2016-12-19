import numpy as np
import cv2
import glob
import random

images_per_calib = 12
num_sensors = 10


def calibrate_sensor(sensor):

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*8,3), np.float32)
    objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    images = glob.glob('sensor%s/*.jpg' % sensor)
    count = 0
    random.shuffle(images)
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (8,6),None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)

            cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, (8,6), corners,ret)
            cv2.imshow('img',img)
            cv2.waitKey(500)
            count += 1

        print("Progress: %f" % (float(images_per_calib * sensor + count) / (images_per_calib * num_sensors)))
        if count >= images_per_calib:
            break


    cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    np.save(open('sensor%s/ret'   % sensor, 'w'), ret)
    np.save(open('sensor%s/mtx'   % sensor, 'w'), mtx)
    np.save(open('sensor%s/dist'  % sensor, 'w'), dist)
    np.save(open('sensor%s/rvecs' % sensor, 'w'), rvecs)
    np.save(open('sensor%s/tvecs' % sensor, 'w'), tvecs)

for sensor in range(num_sensors):
    if sensor == 2:
        continue
    calibrate_sensor(sensor)
