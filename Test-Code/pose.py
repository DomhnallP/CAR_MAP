#!/usr/bin/python
from __future__ import print_function
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from openalpr import Alpr
import cv2

WINDOW_NAME = 'openalpr'
FRAME_SKIP = 2
font = cv2.FONT_HERSHEY_DUPLEX


class image_converter:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "camera/image_raw", Image, self.callback)
        self.alpr = Alpr("eu", "/etc/openalpr/openalpr.conf",
                         "/usr/local/home/u180107/openalpr/runtime_data")
        self.alpr.set_detect_region(True)
        self.currentConfidence = 0
        self.currentDistance = 0
        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        objp = np.float32([
            [0, 0, 0],
            [8, 0, 0],
            [0, 6, 0],
            [8, 6, 0]
            ])
        axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)
        with np.load('calibration.npz') as X:
            mtx, dist = [X[i] for i in ('mtx', 'dist')]

    def draw(img, corners, imgpts):
        corner = tuple(corners[0].ravel())
        img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
        img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
        img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
        return img



   def getPose(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (8, 6), None)
        print("TEST")

        if ret == True:
            result = np.array(
                [corners[0], corners[8], corners[45], corners[-1]])
            for res in result:
                cv2.circle(cv_image, (res[0][0], res[0][1]), 5, (0, 0, 255), 3)

            # print(result)
            corners2 = cv2.cornerSubPix(
                gray, result, (11, 11), (-1, -1), criteria)

            ret, rvecs, tvecs = cv2.solvePnP(
                self.objp, corners2, self.mtx, self.dist, cv2.SOLVEPNP_P3P)

            imgpts, jac = cv2.projectPoints(self.axis, rvecs, tvecs, self.mtx, self.dist)

            img = draw(cv_image, corners2, imgpts)
            cv2.imshow('img', img)

            successful_frames += 1
            cv2.waitKey(10)
        else:
            cv2.imshow('img', cv_image)
            cv2.waitKey(10)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image_labeled = self.getPose(cv_image)
        cv2.imshow(WINDOW_NAME, cv_image_labeled)
        cv2.waitKey(3)

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
