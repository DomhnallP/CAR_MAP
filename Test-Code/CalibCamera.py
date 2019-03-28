#!/usr/bin/env python3
import numpy as np
import cv2
from time import sleep
import os.path

count = 0
scriptPath = os.path.abspath(os.path.dirname(__file__))
cap = cv2.VideoCapture('/usr/local/home/u180107/FYP/Videos/calibrationAttempt2.mp4')

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

while(cap.isOpened()):
    ret, frame = cap.read()
    cropped = frame[720:1440, 0:1280]
    grey = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
    
     
    ret, corners = cv2.findChessboardCorners(grey, (9,6),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(grey,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv2.imwrite(os.path.join(scriptPath, "../dataset/calibData/image%d.jpg" % count), cropped)
        img = cv2.drawChessboardCorners(cropped, (9,6), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(500)
        count +=1

    # cv2.imshow('frame',cropped)
    # if cv2.waitKey(500) & 0xFF == ord('q'):
    #     break
        
cap.release()
cv2.destroyAllWindows()