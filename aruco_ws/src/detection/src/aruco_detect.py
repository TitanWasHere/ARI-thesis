#! /usr/bin/env python
# Detect aruco from realsense camera D435 and publish the image with the aruco detected
# Use ROS1 and Opencv 3.2.0
# 
import rospy
import cv2
from cv2 import aruco
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import copy
from time import sleep
#from image_tools.py import ImageTools

ARUCOBASE_MARKER_ID = 1

# Camera calibration values (RealSense D435)
cx = 421.9996032714844
cy = 237.9039764404297
fx = 616.7684936523438
fy = 615.880615234375
k1 = 0.0
k2 = 0.0
k3 = 0.0
p1 = 0.0
p2 = 0.0

INTRINSIC_CAMERA = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
DISTORTION_CAMERA = np.array([k1, k2, p1, p2, k3])


class detection:
    def __init__(self):
        rospy.init_node('detection')
        self.pub = rospy.Publisher('/camera/aruco', Image, queue_size=1)
        #self.sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback, queue_size=1)
        # Take compressed image and call the 'image_callback' function with the functioning image
        self.sub = rospy.Subscriber('/head_front_camera/color/image_raw/compressed', CompressedImage, self.image_compressed_wrapper, queue_size=1)
        self.bridge = CvBridge()
        self.received_image = False
        self.bridge = CvBridge()

    def image_compressed_wrapper(self, msg):
        image_wrp = np.asarray(bytearray(msg.data), dtype="uint8")
        image_wrp = cv2.imdecode(image_wrp, cv2.IMREAD_COLOR) 
        self.image_callback(image_wrp)

        

    def image_callback(self, msg):
        print('immagine arrivata')
        bridge = CvBridge()
        # self.image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        # image = copy.deepcopy(self.image)
        frame, tvec, rvec = self.get_pose(msg)

        # Publish the image with the aruco detected
        self.pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

    def get_pose(self, frame):
        print('ao')
        cv2.imshow('image', frame)
        cv2.waitKey(1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco.Dictionary_get(aruco.DICT_4X4_50))
        #corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco.Dictionary_get(aruco.DICT_4X4_50))

        if len(corners) > 0:
            ids = ids.flatten()

            rvec, tvec = aruco.estimatePoseSingleMarkers(corners, 0.05, INTRINSIC_CAMERA, DISTORTION_CAMERA)
            frame = cv2.aruco.drawDetectedMarkers(frame, corners,ids)
            print('[INFO] rvec: ', rvec)
            print('[INFO] tvec: ', tvec)
            for i in range(len(ids)):
                frame = cv2.aruco.drawAxis(frame, INTRINSIC_CAMERA, DISTORTION_CAMERA, rvec[i], tvec[i], 0.1)
            #frame = cv2.aruco.drawAxis(frame, INTRINSIC_CAMERA, DISTORTION_CAMERA, rvec, tvec, 0.1)
            return frame, tvec, rvec
        return frame, None, None




if __name__ == '__main__':
    try:
        detection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass  