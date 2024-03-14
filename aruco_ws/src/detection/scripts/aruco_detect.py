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
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation
import copy
from time import sleep

from ArUco_data import *
from camera_data import *
#from image_tools.py import ImageTools

INTRINSIC_CAMERA_REALSENSE_1280 = np.array([[fx_RealSense_1280, 0, cx_RealSense_1280], [0, fy_RealSense_1280, cy_RealSense_1280], [0, 0, 1]])
DISTORTION_CAMERA_REALSENSE_1280 = np.array([k1_RealSense_1280, k2_RealSense_1280, p1_RealSense_1280, p2_RealSense_1280, k3_RealSense_1280])

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

class detection:
    def __init__(self):
        rospy.init_node('detection')
        self.pub_torso = rospy.Publisher('/torso_front_camera/color/aruco', Image, queue_size=2)
        self.pub_front = rospy.Publisher('/head_front_camera/color/aruco', Image, queue_size=2)
        self.pub_myPos = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=2)
        self.sub_torso = rospy.Subscriber('/torso_front_camera/color/image_raw', Image, self.image_callback, queue_size=2)
        self.sub_front = rospy.Subscriber('/head_front_camera/color/image_raw/compressed', CompressedImage, self.image_compressed_callback, queue_size=2)
        self.bridge = CvBridge()
        self.received_image = False
        self.bridge = CvBridge()


    def print_camera_position(self, tvec, rvec, ids):
        #Print the camera position given the tvec and rvec of the aruco
        if ids is None:
            return
        for i in range(len(ids)):
            print('[Camera] position:' + str(self.get_camera_position_from_aruco_position(tvec[i], rvec[i])))

    def image_callback(self, msg):
        #bridge = CvBridge()
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        image = copy.deepcopy(self.image)
        frame, tvec, rvec, ids = self.get_pose(image, 1280)
        cv2.imshow('image_torso', frame)
        cv2.waitKey(1)

        self.manage_frame(frame, tvec, rvec, ids, "torso" )

        

    def image_compressed_callback(self, msg):
        image_wrp = np.asarray(bytearray(msg.data), dtype="uint8")
        image_wrp = cv2.imdecode(image_wrp, cv2.IMREAD_COLOR) 
        #self.image_callback(image_wrp)

        bridge = CvBridge()
        
        frame, tvec, rvec, ids = self.get_pose(image_wrp)
        
        cv2.imshow('image_front', image_wrp)
        cv2.waitKey(1)
        
        self.manage_frame(frame, tvec, rvec, ids, "front")
        #self.pub_torso.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    def get_pose(self, frame, width=848):
        
        #cv2.imshow('image', frame)
        #cv2.waitKey(1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #cv2.imshow('image', gray)  
        #cv2.waitKey(1)
        #corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco.Dictionary_get(aruco.DICT_4X4_50))
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco.Dictionary_get(aruco.DICT_4X4_50))
        if ids is not None:
            print('---------------------------------')
            print('ids: ' + str(ids[0][0]))
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
            for i in range(len(ids)):
                frame = cv2.aruco.drawAxis(frame, INTRINSIC_USED, DISTORTION_USED, rvec[i], tvec[i], 0.1)
                print('[' + str(ids[i]) + ']: { tvec: ' + str(tvec[i][0]) + ' rvec: ' + str(rvec[i][0]) + ' }')

            #frame = cv2.aruco.drawAxis(frame, INTRINSIC_CAMERA, DISTORTION_CAMERA, rvec, tvec, 0.1)
            return frame, tvec, rvec, ids
        return frame, None, None, None

    def manage_frame(self, frame, tvec, rvec, ids, type):
        if ids is not None:
            #self.print_camera_position(tvec, rvec, ids)
            # publish on /initialpose the position of the camera respect to the aruco in the map frame
            for i in range(len(ids)):

                ArUco_in_map = ArUcos[str(ids[i])]["position"]

                # If they aren't, convert each orientation to a quaternion
                # Calculate the qyaternion that represent the rotation of the aruco respect to the camera
                # Calculate the inverse quaternion that rapresent the rotation of the camera respect to the aruco
                # Multiply the quaternion of the aruco respect to the map by the inverse quaternion of the aruco respect to the camera to obtain the quaternion of the camera respect to the map

                
                # Calculate the quaternion that represent the rotation of the aruco respect to the camera
                r = Rotation.from_rotvec(rvec[i][0])
                quat_aruco_camera = r.as_quat()
                # Calculate the inverse quaternion that rapresent the rotation of the camera respect to the aruco
                quat_camera_aruco = np.array([quat_aruco_camera[0], -quat_aruco_camera[1], -quat_aruco_camera[2], -quat_aruco_camera[3]])
                # Multiply the quaternion of the aruco respect to the map by the inverse quaternion of the aruco respect to the camera to obtain the quaternion of the camera respect to the map
                quat_aruco_map = np.array([ArUcos[str(ids[i])]["orientation"][3], ArUcos[str(ids[i])]["orientation"][0], ArUcos[str(ids[i])]["orientation"][1], ArUcos[str(ids[i])]["orientation"][2]])
                quat_camera_map = np.array([quat_aruco_map[0]*quat_camera_aruco[0] - quat_aruco_map[1]*quat_camera_aruco[1] - quat_aruco_map[2]*quat_camera_aruco[2] - quat_aruco_map[3]*quat_camera_aruco[3],
                                            quat_aruco_map[0]*quat_camera_aruco[1] + quat_aruco_map[1]*quat_camera_aruco[0] + quat_aruco_map[2]*quat_camera_aruco[3] - quat_aruco_map[3]*quat_camera_aruco[2],
                                            quat_aruco_map[0]*quat_camera_aruco[2] - quat_aruco_map[1]*quat_camera_aruco[3] + quat_aruco_map[2]*quat_camera_aruco[0] + quat_aruco_map[3]*quat_camera_aruco[1],
                                            quat_aruco_map[0]*quat_camera_aruco[3] + quat_aruco_map[1]*quat_camera_aruco[2] - quat_aruco_map[2]*quat_camera_aruco[1] + quat_aruco_map[3]*quat_camera_aruco[0]])
                # Calculate the position of the camera respect to the aruco
                #camera_position_aruco = self.get_camera_position_from_aruco_position(tvec[i][0], rvec[i][0])
                # Calculate the position of the aruco respect to the map
                aruco_position_map = np.array(ArUcos[str(ids[i][0])]["position"])
                
                # #Publish the position of the camera respect to the aruco in the map frame
                # initpose = PoseWithCovarianceStamped()
                # initpose.header = Header()
                # initpose.header.stamp = rospy.Time.now()
                # initpose.header.frame_id = "map"
                # initpose.pose.pose.position.x = camera_position_in_map[0]
                # initpose.pose.pose.position.y = camera_position_in_map[1]
                # initpose.pose.pose.position.z = camera_position_in_map[2]
                # initpose.pose.pose.orientation.x = camera_quat_in_map[0]
                # initpose.pose.pose.orientation.y = camera_quat_in_map[1]
                # initpose.pose.pose.orientation.z = camera_quat_in_map[2]
                # initpose.pose.pose.orientation.w = camera_quat_in_map[3]
                # self.pub_myPos.publish(initpose)



        if type == "torso":
            self.pub_torso.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        elif type == "front":
            self.pub_front.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
    


if __name__ == '__main__':
    try:
        detection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass  
