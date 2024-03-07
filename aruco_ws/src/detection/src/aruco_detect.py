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

# Camera calibration values (RealSense D435) - 848x480
cx_RealSense = 421.9996032714844
cy_RealSense = 237.9039764404297
fx_RealSense = 616.7684936523438
fy_RealSense = 615.880615234375
k1_RealSense = 0.0
k2_RealSense = 0.0
k3_RealSense = 0.0
p1_RealSense = 0.0
p2_RealSense = 0.0

INTRINSIC_CAMERA_REALSENSE = np.array([[fx_RealSense, 0, cx_RealSense], [0, fy_RealSense, cy_RealSense], [0, 0, 1]])
DISTORTION_CAMERA_REALSENSE = np.array([k1_RealSense, k2_RealSense, p1_RealSense, p2_RealSense, k3_RealSense])

# Camera calibration values (RealSense D435) - 1280x960
cx_RealSense_1280 = 642.2582577578172
cy_RealSense_1280 = 474.1471906434584
fx_RealSense_1280 = 999.461170663331
fy_RealSense_1280 = 996.9611451866272
# k1_RealSense_1280 = 0.164427473610091
# k2_RealSense_1280 = -0.2717244716038656
# k3_RealSense_1280 = 0.0
# p1_RealSense_1280 = -0.002867946281892625
# p2_RealSense_1280 = -9.69782173585606e-05
k1_RealSense_1280 = 0.0
k2_RealSense_1280 = 0.0
k3_RealSense_1280 = 0.0
p1_RealSense_1280 = 0.0
p2_RealSense_1280 = 0.0

INTRINSIC_CAMERA_REALSENSE_1280 = np.array([[fx_RealSense_1280, 0, cx_RealSense_1280], [0, fy_RealSense_1280, cy_RealSense_1280], [0, 0, 1]])
DISTORTION_CAMERA_REALSENSE_1280 = np.array([k1_RealSense_1280, k2_RealSense_1280, p1_RealSense_1280, p2_RealSense_1280, k3_RealSense_1280])

class detection:
    def __init__(self):
        rospy.init_node('detection')
        self.pub_torso = rospy.Publisher('/torso_front_camera/color/aruco', Image, queue_size=2)
        self.pub_front = rospy.Publisher('/head_front_camera/color/aruco', Image, queue_size=2)
        self.sub_torso = rospy.Subscriber('/torso_front_camera/color/image_raw', Image, self.image_callback, queue_size=2)
        self.sub_front = rospy.Subscriber('/head_front_camera/color/image_raw/compressed', CompressedImage, self.image_compressed_callback, queue_size=2)
        self.bridge = CvBridge()
        self.received_image = False
        #self.bridge = CvBridge()

    def image_callback(self, msg):
        bridge = CvBridge()
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        image = copy.deepcopy(self.image)
        frame, tvec, rvec = self.get_pose(image, 1280)
        cv2.imshow('image_torso', frame)
        cv2.waitKey(1)

        self.pub_torso.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

        

    def image_compressed_callback(self, msg):
        image_wrp = np.asarray(bytearray(msg.data), dtype="uint8")
        image_wrp = cv2.imdecode(image_wrp, cv2.IMREAD_COLOR) 
        #self.image_callback(image_wrp)

        bridge = CvBridge()
        
        frame, tvec, rvec = self.get_pose(image_wrp)
        
        cv2.imshow('image_front', image_wrp)
        cv2.waitKey(1)

        self.pub_front.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

    def get_pose(self, frame, width=848):
        #cv2.imshow('image', frame)
        #cv2.waitKey(1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #cv2.imshow('image', gray)  
        #cv2.waitKey(1)
        #corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco.Dictionary_get(aruco.DICT_4X4_50))
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco.Dictionary_get(aruco.DICT_4X4_50))

        if len(corners) > 0:
            ids = ids.flatten()

            if width == 848:
                INTRINSIC_USED = INTRINSIC_CAMERA_REALSENSE
                DISTORTION_USED = DISTORTION_CAMERA_REALSENSE
            else:
                INTRINSIC_USED = INTRINSIC_CAMERA_REALSENSE_1280
                DISTORTION_USED = DISTORTION_CAMERA_REALSENSE_1280

            rvec, tvec = aruco.estimatePoseSingleMarkers(corners, 0.05, INTRINSIC_USED, DISTORTION_USED)
            frame = cv2.aruco.drawDetectedMarkers(frame, corners,ids)
            print('[INFO] rvec: ', rvec)
            print('[INFO] tvec: ', tvec)
            for i in range(len(ids)):
                frame = cv2.aruco.drawAxis(frame, INTRINSIC_USED, DISTORTION_USED, rvec[i], tvec[i], 0.1)
            #frame = cv2.aruco.drawAxis(frame, INTRINSIC_CAMERA, DISTORTION_CAMERA, rvec, tvec, 0.1)
            return frame, tvec, rvec
        return frame, None, None




if __name__ == '__main__':
    try:
        detection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass  