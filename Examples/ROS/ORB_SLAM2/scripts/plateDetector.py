#!/usr/bin/env python3
from __future__ import print_function
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from openalpr import Alpr
import cv2

WINDOW_NAME  = 'openalpr'
FRAME_SKIP = 2
font = cv2.FONT_HERSHEY_DUPLEX

class image_converter:
  def __init__(self):

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.callback)
    self.alpr = Alpr("eu", "/etc/openalpr/openalpr.conf", "/usr/local/home/u180107/openalpr/runtime_data")
    self.alpr.set_detect_region(True)
    self.currentConfidence = 0
    self.currentDistance = 0

  def distanceByArea(self, cv_image):

    results = self.alpr.recognize_ndarray(cv_image)
    for i, plate in enumerate(results['results']):
        best_candidate = plate['coordinates']
        corner1 = (best_candidate[0]['x'] , best_candidate[0]['y'])
        corner2 = (best_candidate[2]['x'] , best_candidate[2]['y'])
        platePixelWidth = abs(int(best_candidate[2]['x']) - int(best_candidate[0]['x']))
        platePixelHeight = abs(int(best_candidate[2]['y']) - int(best_candidate[0]['y']))
        platePixelArea = platePixelHeight*platePixelWidth
        
        # The distance equation required that the camera be calibrated, currently they're 'close enough'
        distance = round(52000*pow(platePixelArea, -0.368), 2)
        detected = plate['candidates'][0]
        if self.currentConfidence < detected['confidence'] or abs(distance-self.currentDistance)>200:
            self.currentConfidence = detected['confidence']
            # print('Plate #{}: {:7s} ({:.2f}%) - Plate distance: {}mm'.format(i, detected['plate'].upper(), detected['confidence'],distance))
        self.currentDistance = distance
        
        cv2.rectangle(cv_image,corner1,corner2,(0,255,0),2)
        cv2.putText(cv_image, ('distance = %dmm' % distance), (i*330,460), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

    return cv_image

  def distanceByHomography(self, cv_image):
    cam_mtx = np.array([
      [767.92632836, 0 , 325.7824046],
      [0, 770.98907555, 157.44636998],
      [0, 0, 1]
    ])
    dist_coeffs = np.zeros((5, 1))
    results = self.alpr.recognize_ndarray(cv_image)
    for i, plate in enumerate(results['results']):
        best_candidate = plate['coordinates']
        worldPoints = np.float32([[0, 0, 0], [510, 0, 0], [510,110, 0], [0, 110, 0]])
        imagePoints = np.float32([
          (
            best_candidate[a]['x'] , best_candidate[a]['y']
          ) for a in range(0,4)])
        # print(imgPoints)
        # print(worldPoints)
        imgPoints = np.ascontiguousarray(imagePoints[:,:2].reshape(4,1,2))
        _ret, rvec, tvec = cv2.solvePnP(worldPoints, imgPoints, cam_mtx, dist_coeffs, flags=cv2.SOLVEPNP_P3P)
        print("Rotation Vector:\n {0}".format(rvec))
        print("Translation Vector:\n {0}".format(tvec))
 
        end_point2D, jacobian = cv2.projectPoints(np.array([(0.0, 0.0, 1000.0)]), rvec, tvec, cam_mtx, dist_coeffs)
        
        for p in imagePoints:
            cv2.circle(cv_image, (int(p[0]), int(p[1])), 3, (0,0,255), -1)
        
        
        p1 = ( int(imagePoints[0][0]), int(imagePoints[0][1]))
        p2 = ( int(end_point2D[0][0][0]), int(end_point2D[0][0][1]))
        
        cv2.line(cv_image, p1, p2, (255,0,0), 2)

    return cv_image

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      # cv2.imshow(WINDOW_NAME, cv_image)
    except CvBridgeError as e:
      print(e)

    # cv_image_labeled = self.distanceByArea(cv_image)
    cv_image_labeled = self.distanceByHomography(cv_image)

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