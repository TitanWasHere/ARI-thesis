# Given a topic from a realsense camera D345 '/camera/camera/color/image_raw'
# Using ROS1 and Opencv 3.2.0, take the image from the topic and detect the aruco markers in the image.
# 

import rospy
import cv2
from cv2 import aruco
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import copy
from scipy.spatial.transform import Rotation as R
from time import sleep


SHOW_IMG = False
ARUCO_SIZE = 0.07
ARUCOBASE_X_OFFSET_FROM_BASELINK = 0.0
ARUCOBASE_Y_OFFSET_FROM_BASELINK = -0.045
ARUCOBASE_Z_OFFSET_FROM_BASELINK = -0.085
ARUCOBASE_MARKER_ID = 1
ARUCOPUPPET_MARKER_ID = 0
LOCALIZE_PUPPET = True

# Camera calibration values (RealSense D435)
cx = 958.620910644531
cy = 564.277893066406
fx = 1376.80395507812
fy = 1376.80322265625
k1 = 0.0
k2 = 0.0
k3 = 0.0
p1 = 0.0
p2 = 0.0

INTRINSIC_CAMERA = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
DISTORTION_CAMERA = np.array([k1, k2, p1, p2, k3])

rot_x = np.array([
      [1, 0, 0], 
      [0, 0, 1], 
      [0, -1, 0]
      ], dtype=np.float64)

rot_z = np.array([  
      [1, 0, 0], 
      [0, 1, 0], 
      [0, 0, 1]
      ], dtype=np.float64)

DEFAULT_ROT = np.dot(rot_x, rot_z)

pub = rospy.Publisher('/camera/aruco', Image, queue_size=1)

def image_callback(msg):
  print('Received image')
  bridge = CvBridge()
  frame = bridge.imgmsg_to_cv2(msg, "bgr8")
  gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
  aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
  parameters = aruco.DetectorParameters_create()
  corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
  if ids is not None:
    for i in range(len(ids)):
      if ids[i] == ARUCOPUPPET_MARKER_ID:
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], ARUCO_SIZE, INTRINSIC_CAMERA, DISTORTION_CAMERA)
        aruco.drawAxis(frame, INTRINSIC_CAMERA, DISTORTION_CAMERA, rvec, tvec, 0.1)
        aruco.drawDetectedMarkers(frame, corners)
        print('rvec: ', rvec)
        print('tvec: ', tvec)
        print('ids: ', ids)
        print('corners: ', corners)
        print('rejectedImgPoints: ', rejectedImgPoints)
        print('frame: ', frame)
        cv2.imshow('frame', frame)
        cv2.waitKey(1)
      else:
        print('No Aruco markers found')
  else:
    print('No Aruco markers found')
  
  pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))


def main():
  rospy.init_node('aruco_detector')
  bridge = CvBridge()
  sub = rospy.Subscriber('/camera/camera/color/image_raw', Image, image_callback, queue_size=1)
  rospy.spin()

if __name__ == '__main__':
  main()