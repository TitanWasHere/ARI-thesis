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
import copy
#from pal_navigation_msgs.msg import GoToPOIActionGoal
from time import sleep
from pyquaternion import Quaternion
from std_msgs.msg import String
from ArUco_data import *
from camera_data import *
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
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
        rospy.init_node('detection', anonymous=True)
        self.pub_torso = rospy.Publisher('/torso_front_camera/color/aruco', Image, queue_size=2)
        self.pub_front = rospy.Publisher('/head_front_camera/color/aruco', Image, queue_size=2)
        self.pub_myPos = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=2)
        self.sub_torso = rospy.Subscriber('/torso_front_camera/color/image_raw', Image, self.image_callback, queue_size=2)
        self.sub_front = rospy.Subscriber('/head_front_camera/color/image_raw/compressed', CompressedImage, self.image_compressed_callback, queue_size=2)
        self.listener = tf.TransformListener()
        self.sub_tf = rospy.Subscriber('/tf', TransformStamped, self.transform_callback)

        self.bridge = CvBridge()
        self.received_image = False

        self.map_to_odom = None
        self.odom_to_baseFootprint = None
        self.baseFootprint_to_camera = None
        self.map_to_aruco = None
        self.map_to_baseFootprint = None


        # self.goal = rospy.Publisher('/poi_navigation_server/go_to_poi/goal', GoToPOIActionGoal, queue_size=2)
        # self.move()

    # def move(self):
    #     self.goal_msg = GoToPOIActionGoal()
    #     self.goal_msg.header.seq = 0
    #     self.goal_msg.header.stamp.secs = 0
    #     self.goal_msg.header.stamp.nsecs = 0
    #     self.goal_msg.header.frame_id = 'map'
    #     self.goal_msg.goal_id.stamp.secs = 0
    #     self.goal_msg.goal_id.stamp.nsecs = 0
    #     self.goal_msg.goal_id.id = ''
    #     self.goal_msg.goal.poi.data = 'ari_16c_dockstation'
    #     self.goal.publish(self.goal_msg)


    def transform_callback(self, msg):
        trans, rot = self.get_transform("base_footprint", "torso_front_camera_color_frame")
        self.baseFootprint_to_camera = {"position": np.array([trans[0], trans[1], trans[2]]) , "orientation": np.array([rot[3], rot[0], rot[1], rot[2]])} 
        trans, rot = self.get_transform("odom", "base_footprint")
        self.odom_to_baseFootprint = {"position": np.array([trans[0], trans[1], trans[2]]) , "orientation": np.array([rot[3], rot[0], rot[1], rot[2]])} 
        trans, rot = self.get_transform("map", "odom")
        self.map_to_odom = {"position": np.array([trans[0], trans[1], trans[2]]) , "orientation": np.array([rot[3], rot[0], rot[1], rot[2]])} 
        trans, rot = self.get_transform("map", "aruco")
        self.map_to_aruco = {"position": np.array([trans[0], trans[1], trans[2]]) , "orientation": np.array([rot[3], rot[0], rot[1], rot[2]])} 
        #print("map_to_aruco: " + str(self.map_to_aruco))

        trans, rot = self.get_transform("map", "base_footprint")
        self.map_to_baseFootprint = {"position": np.array([trans[0], trans[1], trans[2]]) , "orientation": np.array([rot[3], rot[0], rot[1], rot[2]])} 
        
    def get_transform(self, frame_x, frame_y):
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform(frame_x, frame_y, now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform(frame_x, frame_y, now)
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Unable to lookup transform from /%s to /%s: %s", frame_x, frame_y, e)



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
            for i in range(len(ids)):
                try:
                    aruco = ArUcos[str(ids[i])]

                    while self.map_to_odom is None or self.odom_to_baseFootprint is None or self.baseFootprint_to_camera is None or self.map_to_aruco is None or self.map_to_baseFootprint is None:
                        print('Waiting for tf...')
                        sleep(1)

                    #base_to_camera, base_to_odom, map_to_odom, map_to_aruco = self.get_tf2_positions()
                    camera_T_aruco = np.zeros((4,4))
                    camera_T_aruco[0:3, 0:3] = cv2.Rodrigues(rvec[i])[0]
                    camera_T_aruco[0:3, 3] = tvec[i]
                    camera_T_aruco[3, 3] = 1

                    # quat_odom_to_base = Quaternion(np.array([self.odom_to_baseFootprint["orientation"][0], self.odom_to_baseFootprint["orientation"][1], self.odom_to_baseFootprint["orientation"][2], self.odom_to_baseFootprint["orientation"][3]]))
                    # odom_T_base = np.zeros((4,4))
                    # odom_T_base[0:3, 0:3] = quat_odom_to_base.rotation_matrix
                    # odom_T_base[0:3, 3] = np.array([self.odom_to_baseFootprint["position"][0], self.odom_to_baseFootprint["position"][1], self.odom_to_baseFootprint["position"][2]])
                    # odom_T_base[3, 3] = 1



                    #quat_orientation_base_to_camera = Quaternion(np.array([base_to_camera.transform.rotation.w, base_to_camera.transform.rotation.x, base_to_camera.transform.rotation.y, base_to_camera.transform.rotation.z]))
                    quat_orientation_base_to_camera = Quaternion(np.array([self.baseFootprint_to_camera["orientation"][0], self.baseFootprint_to_camera["orientation"][1], self.baseFootprint_to_camera["orientation"][2], baseFootprint_to_camera["orientation"][3]])) 
                    base_T_camera = np.zeros((4,4))
                    base_T_camera[0:3, 0:3] = quat_orientation_base_to_camera.rotation_matrix
                    #base_T_camera[0:3, 3] = np.array([base_to_camera.transform.translation.x, base_to_camera.transform.translation.y, base_to_camera.transform.translation.z])
                    base_T_camera[0:3, 3] = np.array([self.baseFootprint_to_camera["position"][0], self.baseFootprint_to_camera["position"][1], self.baseFootprint_to_camera["position"][2]])
                    base_T_camera[3, 3] = 1

                    # base_T_aruco = np.dot(odom_T_base, base_T_camera)
                    base_T_aruco = np.dot(base_T_camera, camera_T_aruco)
                    aruco_T_base = np.linalg.inv(base_T_aruco)

                    

                    #quat_map_aruco = Quaternion(np.array([map_to_aruco.transform.rotation.w, map_to_aruco.transform.rotation.x, map_to_aruco.transform.rotation.y, map_to_aruco.transform.rotation.z]))
                    quat_map_aruco = Quaternion(np.array([self.map_to_aruco["orientation"][0], self.map_to_aruco["orientation"][1], self.map_to_aruco["orientation"][2], self.map_to_aruco["orientation"][3]]))
                    map_T_aruco = np.zeros((4,4))
                    map_T_aruco[0:3, 0:3] = quat_map_aruco.rotation_matrix
                    #map_T_aruco[0:3, 3] = np.array([map_to_aruco.transform.translation.x, map_to_aruco.transform.translation.y, map_to_aruco.transform.translation.z])
                    map_T_aruco[0:3, 3] = np.array([self.map_to_aruco["position"][0], self.map_to_aruco["position"][1], self.map_to_aruco["position"][2]])
                    map_T_aruco[3, 3] = 1

                    # quat_map_aruco = Quaternion(aruco["orientation"])
                    # map_T_aruco = np.zeros((4,4))
                    # map_T_aruco[0:3, 0:3] = quat_map_aruco.rotation_matrix
                    # map_T_aruco[0:3, 3] = np.array(aruco["position"])
                    # map_T_aruco[3, 3] = 1  

                    map_T_base = np.dot(map_T_aruco, aruco_T_base)
                    quat_map_base = Quaternion(matrix=map_T_base[0:3, 0:3])


                    ## PROVA ##
                    print(self.map_to_baseFootprint)
                    quat_map_basefootprint = Quaternion(np.array([self.map_to_baseFootprint["orientation"][0], self.map_to_baseFootprint["orientation"][1], self.map_to_baseFootprint["orientation"][2], self.map_to_baseFootprint["orientation"][3]]))
                    print("quat_map_basefootprint: " + str(quat_map_basefootprint))
                    ###########
                    
                    pose = PoseWithCovarianceStamped()
                    pose.header = Header()
                    pose.header.stamp = rospy.Time.now()
                    pose.header.frame_id = "map"
                    pose.pose.pose.position.x = map_T_base[0, 3]
                    pose.pose.pose.position.y = map_T_base[1, 3]
                    pose.pose.pose.position.z = map_T_base[2, 3]
                    pose.pose.pose.orientation.x = quat_map_basefootprint[1]
                    pose.pose.pose.orientation.y = quat_map_basefootprint[2]
                    pose.pose.pose.orientation.z = quat_map_basefootprint[3]
                    pose.pose.pose.orientation.w = quat_map_basefootprint[0]

                    #self.pub_myPos.publish(pose)
                    print("pose: " + str(pose))


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
