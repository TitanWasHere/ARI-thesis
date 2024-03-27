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
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation
import copy
from time import sleep
from pyquaternion import Quaternion

from ArUco_data import *
from camera_data import *
import tf2_ros
import tf
import tf2_geometry_msgs 
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

    def quaternion_multiplication(self, q0, q1):
        # Multiplication of two quaternions
        x0, y0, z0, w0 = q0
        x1, y1, z1, w1 = q1
        x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
        y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
        z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
        w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
        return np.array([x, y, z, w])
    
    def manage_frame(self, frame, tvec, rvec, ids, type):
        if ids is not None:
            for i in range(len(ids)):
                try:
                    aruco = ArUcos[str(ids[i])]

                    camera_pos = position_base_footprint_camera
                    camera_ori = np.array(orientation_base_footprint_camera) # expressed in quaternion

                    # Trasformation matrix from odom to camera
                    camera_ori_matrix = Quaternion(matrix=camera_ori).rotation_matrix
                    odom_T_camera = np.zeros((4, 4))
                    odom_T_camera[:3, :3] = camera_ori_matrix
                    odom_T_camera[:3, 3] = camera_pos
                    odom_T_camera[3, 3] = 1

                    # Trasformation matrix from camera to aruco
                
                    rvec_mat = cv2.Rodrigues(rvec[i])[0]
                    camera_T_aruco = np.zeros((4,4))
                    camera_T_aruco[:3, :3] = rvec_mat
                    camera_T_aruco[:3, 3] = tvec[i]
                    camera_T_aruco[3, 3] = 1

                    odom_T_aruco = np.dot(odom_T_camera, camera_T_aruco)

                    aruco_T_odom = np.linalg.inv(odom_T_aruco)

                    aruco_pos_in_map = aruco['position']
                    aruco_ori_in_map = aruco['orientation']
                    aruco_ori_in_map = Quaternion(matrix=aruco_ori_in_map).rotation_matrix

                    map_T_aruco = np.zeros((4, 4))
                    map_T_aruco[:3, :3] = aruco_ori_in_map
                    map_T_aruco[:3, 3] = aruco_pos_in_map
                    map_T_aruco[3, 3] = 1

                    map_T_odom = np.dot(map_T_aruco, aruco_T_odom)

                    tx = map_T_odom[0, 3]
                    ty = map_T_odom[1, 3]
                    tz = map_T_odom[2, 3]

                    quat = Quaternion(matrix=map_T_odom[:3, :3])
                    qx = quat[0]
                    qy = quat[1]
                    qz = quat[2]
                    qw = quat[3]

                    new_pose = PoseWithCovarianceStamped()
                    new_pose.header = Header()
                    new_pose.header.stamp = rospy.Time.now()
                    new_pose.header.frame_id = "map"
                    new_pose.pose.pose.position.x = tx
                    new_pose.pose.pose.position.y = ty
                    new_pose.pose.pose.position.z = tz
                    new_pose.pose.pose.orientation.x = qx
                    new_pose.pose.pose.orientation.y = qy
                    new_pose.pose.pose.orientation.z = qz
                    new_pose.pose.pose.orientation.w = qw
                    self.pub_myPos.publish(new_pose)

                except KeyError:
                    print('Aruco not found')
                    return


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
