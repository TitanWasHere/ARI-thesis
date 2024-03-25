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
from pyquaternion import Quaternion

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
        
        #cv2.imshow('image_front', image_wrp)
        #cv2.waitKey(1)
        
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
            #self.print_camera_position(tvec, rvec, ids)
            # publish on /initialpose the position of the camera respect to the aruco in the map frame
            for i in range(len(ids)):
                try:
                    aruco = ArUcos[str(ids[i])]
                    rvec_aruco_from_cam = rvec[i][0]
                    tvec_aruco_from_cam = tvec[i][0]


                    # # -------------- PROVA ----------------
                    # R = cv2.Rodrigues(rvec_aruco_from_cam)[0]
                    # #print('R: ' + str(R))
                    # #print('tvec_aruco_from_cam: ' + str(tvec_aruco_from_cam))
                    # #print('tvec_aruco_from_cam: ' + str(tvec_aruco_from_cam))

                    # R_T = np.transpose(R)
                    # T = np.transpose(tvec_aruco_from_cam)

                    # xyz = np.dot(-R_T, T)
                    # x, y, z = xyz
                    

                    # print('x: ' + str(x) + ' y: ' + str(y) + ' z: ' + str(z))

                    # # Transform R_T and T in transformation matrix
                    # T_new = np.zeros((4,4))
                    # T_new[:3,:3] = R_T
                    # T_new[:3,3] = T
                    # T_new[3,3] = 1

                    # # ------------------------------------


                    # Rvec to matrix 3x3 with rodrigues
                    rvec_matrix = cv2.Rodrigues(rvec_aruco_from_cam)[0]

                    orient_zero = np.array([0, 0, 0, 1])
                    pose_zero = np.array([0, 0, 0])
                    orient_zero = (Quaternion(orient_zero)).rotation_matrix
                    zero_mat = np.zeros((4,4))
                    zero_mat[3,3] = 1
                    zero_mat[:3,:3] = orient_zero
                    zero_mat[:3,3] = pose_zero

                    # create rotation matrix 4x4
                    T_cam = np.zeros((4,4))
                    T_cam[:3,:3] = rvec_matrix
                    T_cam[3,3] = 1
                    #print(T_cam[:3, 3])
                    T_cam[:3,3] = tvec_aruco_from_cam
                    
                    # Inverse of rot_matrix
                    T_cam = np.linalg.inv(T_cam)
                    tvec_cam = T_cam[:3, 3]
                    rvec_cam = T_cam[:3, :3]
                    q_cam = Quaternion(matrix=np.array(rvec_cam))


                    # Create the transformation matrix of the aruco
                    T_aruco = np.zeros((4,4))
                    # aruco["orientation"] is a quaternion
                    quat_aruco = np.array(aruco["orientation"])
                    quat_aruco = Quaternion(quat_aruco)
                    T_aruco[:3,:3] = quat_aruco.rotation_matrix
                    T_aruco[3,3] = 1
                    T_aruco[:3,3] = np.array(aruco["position"])

                    # Create the transformation matrix of the camera respect to the aruco
                    #T_cam_aruco = np.dot(zero_mat, T_aruco)
                    
                    # QUESTA DOVREBBE ESSERE QUELLA PIU GIUSTA
                    T_cam_aruco = np.dot(T_aruco, T_cam)

                    #T_cam_aruco = np.dot(T_aruco, T_new)


                    # Extract the position and orientation of the camera respect to the aruco
                    tvec_final = T_cam_aruco[:3,3]
                    rvec_final = np.eye(3)
                    rvec_final = T_cam_aruco[:3,:3]
                    
                    rvec_final = np.array(rvec_final)
                    #print('rvec_final: ' + str(rvec_final))
                    q_final = Quaternion(matrix=rvec_final)
                    print('tvec_final: ' + str(tvec_final))

                    # Publish the position of the camera respect to the aruco
                    pose = PoseWithCovarianceStamped()
                    pose.header = Header()
                    pose.header.stamp = rospy.Time.now()
                    pose.header.frame_id = "map"
                    pose.pose.pose.position.x = tvec_final[0]
                    pose.pose.pose.position.y = tvec_final[1]
                    pose.pose.pose.position.z = tvec_final[2]
                    pose.pose.pose.orientation.x = q_final[0]
                    pose.pose.pose.orientation.y = q_final[1]
                    pose.pose.pose.orientation.z = q_final[2]
                    pose.pose.pose.orientation.w = q_final[3]
                    self.pub_myPos.publish(pose) 


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
