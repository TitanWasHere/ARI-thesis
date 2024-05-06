#! /usr/bin/env python

import rospy
import cv2
from cv2 import aruco
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped
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
from geometry_msgs.msg import TransformStamped, Twist
import tf2_geometry_msgs 
from collections import namedtuple
import geometry_msgs.msg
from tf2_msgs.msg import TFMessage

INTRINSIC_CAMERA_REALSENSE_1280 = np.array([[fx_RealSense_1280, 0, cx_RealSense_1280], [0, fy_RealSense_1280, cy_RealSense_1280], [0, 0, 1]])
DISTORTION_CAMERA_REALSENSE_1280 = np.array([k1_RealSense_1280, k2_RealSense_1280, p1_RealSense_1280, p2_RealSense_1280, k3_RealSense_1280])


class calibration:


    def __init__(self):
        

        self.pub_torso = rospy.Publisher('/torso_front_camera/color/aruco', Image, queue_size=2)
        self.pub_front = rospy.Publisher('/head_front_camera/color/aruco', Image, queue_size=2)
        self.pub_myPos = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=2)
        self.pub_velocity = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=2)
        
        self.sub_torso = rospy.Subscriber('/torso_front_camera/color/image_raw', Image, self.image_callback, queue_size=2)
        
        self.sub_front = rospy.Subscriber('/head_front_camera/color/image_raw/compressed', CompressedImage, self.image_compressed_callback, queue_size=2)
        
        self.sub_transform = rospy.Subscriber("/transform", TFMessage, self.transform_callback)
        self.sub_velocity = rospy.Subscriber("/mobile_base_controller/cmd_vel", Twist, self.velocity_callback)
        
        
        #self.listener = tf.TransformListener()
        
        # Subscribing to /tf to get the transformation between the frames it's a big problem because the function is never called for some reason
        #self.sub_tf = rospy.Subscriber('/tf', TFMessage, self.transform_callback)

        

        self.bridge = CvBridge()
        self.received_image = False

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.map_to_odom = None
        self.odom_to_baseFootprint = None
        self.baseFootprint_to_camera = None
        self.map_to_aruco = {}
        self.map_to_baseFootprint = None
        self.odom_to_camera = None
        self.map_to_camera = None
        self.map_to_headCamera = None

        self.velocity = Twist()
        self.last_velocity = Twist()
        self.last_seen_aruco = None
        self.needToStop = False
        self.time_to_pass_since_last = 10
        self.timeToPassSinceStopped = 2
        self.last_time_seen_aruco = 0
        self.needToRecalibrate = False
    
        #self.can_restart_moving = True

    
    def velocity_callback(self, msg):
        self.velocity = msg

        
    def print_values(self, frame_x, frame_y, val):
        print("Frame: " + frame_x)
        print(" - child_frame: " + frame_y)
        print(" \t - translation ")
        print(" \t \t - x: " + str(val.position.x))
        print(" \t \t - y: " + str(val.position.y))
        print(" \t \t - z: " + str(val.position.z))
        print(" \t - rotation ")
        print(" \t \t - x: " + str(val.orientation.x))
        print(" \t \t - y: " + str(val.orientation.y))
        print(" \t \t - z: " + str(val.orientation.z))
        print(" \t \t - w: " + str(val.orientation.w))




    def transform_callback(self, msg):
        myTransforms = []
        for transforms in msg.transforms:
            myTransforms.append(transforms)

        self.map_to_odom = self.assign_values(myTransforms[0])
        self.odom_to_baseFootprint = self.assign_values(myTransforms[1])
        self.baseFootprint_to_camera = self.assign_values(myTransforms[2])
        self.map_to_baseFootprint = self.assign_values(myTransforms[3])
        self.odom_to_camera = self.assign_values(myTransforms[4])
        self.map_to_camera = self.assign_values(myTransforms[5])
        self.map_to_headCamera = self.assign_values(myTransforms[6])



        i = 0
        for key in ArUcos:
            name = "aruco_" + key
            self.map_to_aruco[name] = self.assign_values(myTransforms[7 + i])
            i+=1
            #print("[" + name + "]: " + str(self.map_to_aruco[name]))
            #self.print_values("map", name, self.map_to_aruco[name])



    def assign_values(self, values):
        # Assign the values to the namedtuple so we can get poses like "map_to_odom.position.x" for better readability
        var = namedtuple("Pose", ["position", "orientation"])
        position = namedtuple("Position", ["x", "y", "z"])
        orientation = namedtuple("Orientation", ["x", "y", "z", "w"])
        pos = position(values.transform.translation.x, values.transform.translation.y, values.transform.translation.z)
        ori = orientation(values.transform.rotation.x, values.transform.rotation.y, values.transform.rotation.z, values.transform.rotation.w)
        return var(pos, ori)
        


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
        
        if self.needToStop and rospy.Time.now() - self.last_time < rospy.Duration(self.timeToPassSinceStopped):
            return frame, None, None, None

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
                #print('[' + str(ids[i]) + ']: { tvec: ' + str(tvec[i][0]) + ' rvec: ' + str(rvec[i][0]) + ' }')frame

            #frame = cv2.aruco.drawAxis(frame, INTRINSIC_CAMERA, DISTORTION_CAMERA, rvec, tvec, 0.1)
            return frame, tvec, rvec, ids
        return frame, None, None, None

    def velocity_to_zero(self):
        velocity = Twist()
        velocity.linear.x = 0
        velocity.linear.y = 0
        velocity.linear.z = 0
        velocity.angular.x = 0
        velocity.angular.y = 0
        velocity.angular.z = 0
        return velocity
    
    def manage_frame(self, frame, tvec, rvec, ids, type):
        #self.publish_arucos_in_map()
        
        #print("after transform_callback ")
        if ids is not None:

            for i in range(len(ids)):
                
                # if self.last_seen_aruco != ids[i] or rospy.Time.now() - self.last_time > self.time_to_pass_since_last:
                #     self.last_velocity = self.velocity
                #     stop_velocity = self.velocity_to_zero()
                #     self.pub_velocity.publish(stop_velocity)
                #     self.needToStop = True
                #     self.needToRecalibrate = True
                #     return
                # else:
                #     my_velocity = self.last_velocity
                #     self.needToStop = False
                #     if self.needToRecalibrate:
                #         self.needToRecalibrate = False
                #         self.last_time = rospy.Time.now()
                #         self.last_seen_aruco = ids[i]
                    
                if not self.needToStop:
                    if self.last_seen_aruco != ids[i] or rospy.Time.now() - self.last_time > rospy.Duration(self.time_to_pass_since_last):
                        self.last_velocity = self.velocity
                        stop_velocity = self.velocity_to_zero()
                        self.pub_velocity.publish(stop_velocity)
                        self.last_time = rospy.Time.now()
                        self.needToStop = True
                        self.last_seen_aruco = ids[i]
                        return
                    
                        



                try:
                    # aruco = ArUcos[str(ids[i])]
                    # sleep(2)
                    arucoKey = "aruco_" + str(ids[i])

                    ok = False
                    for key in ArUcos:
                        if key == str(ids[i]):
                            ok = True
                            break
                    if not ok:
                        print("Aruco not in the list")
                        return
                    
                    while self.map_to_odom is None or self.odom_to_baseFootprint is None or self.baseFootprint_to_camera is None or self.map_to_aruco[arucoKey] is None :
                        print('Waiting for tf...')
                        # print(self.map_to_odom)
                        # print(self.odom_to_baseFootprint)
                        # print(self.baseFootprint_to_camera)
                        # print(self.map_to_baseFootprint)
                        # print(self.map_to_aruco[arucoKey])
                        sleep(1)
                    
                    if type == "front":
                        cam = self.map_to_headCamera
                    else:
                        cam = self.map_to_camera

                    #print("tvec: " + str(tvec[i][0]))
                    distance = self.calculate_distance(tvec[i][0], [0.1, 0.1])
                    print(distance)
                    # Solo se la distanza dall'aruco visto e' minore di 1.2 metri procedo, cosi' da avere maggiore accuratezza
                    if distance < 1.5:

                        map_camera_ori_quat = Quaternion(a = cam.orientation.w, b = cam.orientation.x, c = cam.orientation.y, d = cam.orientation.z)
                        map_T_camera = np.zeros((4,4))
                        map_T_camera[0:3, 0:3] = map_camera_ori_quat.rotation_matrix
                        map_T_camera[0:3, 3] = np.array([cam.position.x, cam.position.y, cam.position.z])
                        map_T_camera[3, 3] = 1

                        camera_T_aruco = np.zeros((4,4))
                        camera_T_aruco[0:3, 0:3] = cv2.Rodrigues(rvec[i])[0]
                        camera_T_aruco[0:3, 3] = tvec[i][0]
                        camera_T_aruco[3, 3] = 1
                        
                        map_T_arucoDetected = np.dot(map_T_camera, camera_T_aruco)

                        self.publish_tf2("map", "detected_aruco", map_T_arucoDetected)


                        baseFootprint_T_camera = np.zeros((4,4))
                        baseFootprint_T_camera[0:3, 0:3] = Quaternion(a = self.baseFootprint_to_camera.orientation.w, b = self.baseFootprint_to_camera.orientation.x, c = self.baseFootprint_to_camera.orientation.y, d = self.baseFootprint_to_camera.orientation.z).rotation_matrix
                        baseFootprint_T_camera[0:3, 3] = np.array([self.baseFootprint_to_camera.position.x, self.baseFootprint_to_camera.position.y, self.baseFootprint_to_camera.position.z])
                        baseFootprint_T_camera[3, 3] = 1

                        baseFootprint_T_aruco = np.dot(baseFootprint_T_camera, camera_T_aruco)

                        aruco_T_baseFootprint = np.linalg.inv(baseFootprint_T_aruco)

                        map_T_aruco = np.zeros((4,4))
                        map_T_aruco[0:3, 0:3] = Quaternion(a = self.map_to_aruco[arucoKey].orientation.w, b = self.map_to_aruco[arucoKey].orientation.x, c = self.map_to_aruco[arucoKey].orientation.y, d = self.map_to_aruco[arucoKey].orientation.z).rotation_matrix
                        map_T_aruco[0:3, 3] = np.array([self.map_to_aruco[arucoKey].position.x, self.map_to_aruco[arucoKey].position.y, self.map_to_aruco[arucoKey].position.z])
                        map_T_aruco[3, 3] = 1

                        map_T_baseFootprint = np.dot(map_T_aruco, aruco_T_baseFootprint)
                        quat_map_base = Quaternion(matrix=map_T_baseFootprint[:3, :3])
                        #self.publish_tf2("map", "new_base", map_T_baseFootprint)


                        pose = PoseWithCovarianceStamped()
                        pose.header = Header()
                        pose.header.stamp = rospy.Time.now()
                        pose.header.frame_id = "map"
                        pose.pose.pose.position.x = map_T_baseFootprint[0, 3]
                        pose.pose.pose.position.y = map_T_baseFootprint[1, 3]
                        pose.pose.pose.position.z =  0 #map_T_baseFootprint[2, 3]
                        pose.pose.pose.orientation.x = quat_map_base[1]
                        pose.pose.pose.orientation.y = quat_map_base[2]
                        pose.pose.pose.orientation.z = quat_map_base[3]
                        pose.pose.pose.orientation.w = quat_map_base[0]
                        print("Ripubblico la posizione")
                        self.pub_myPos.publish(pose)

                        self.pub_velocity.publish(self.last_velocity)
                        self.needToStop = False
                        self.last_time_seen_aruco = rospy.Time.now()
                        self.last_seen_aruco = ids[i]

    
                except KeyError:
                    print('Aruco not found')
                    return


        if type == "torso":
            self.pub_torso.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        elif type == "front":
            self.pub_front.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))


    def calculate_distance(self, tvec, aruco_size):
        # Calcola la norma del vettore di traslazione
        tvec_norm = np.linalg.norm(tvec)
        
        # Calcola l'angolo di inclinazione dell'Aruco rispetto alla telecamera
        alpha = np.arctan2(tvec[1], tvec[2])
        
        # Calcola la distanza utilizzando la trigonometria  
        distance = tvec_norm * np.cos(alpha)
        
        # Correggi la distanza in base alle dimensioni dell'Aruco
        # Supponiamo che le dimensioni dell'Aruco siano date in metri
        aruco_height = aruco_size[0]
        #distance -= aruco_height / 2.0
        
        return distance



    def publish_tf2(self, frame_parent, frame_child, matrix):
            
            br = tf2_ros.TransformBroadcaster()
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = frame_parent
            t.child_frame_id = frame_child
            t.transform.translation.x = matrix[0, 3]
            t.transform.translation.y = matrix[1, 3]
            t.transform.translation.z = matrix[2, 3]
            q = Quaternion(matrix=matrix[:3, :3])
            t.transform.rotation.x = q.x
            t.transform.rotation.y = q.y
            t.transform.rotation.z = q.z
            t.transform.rotation.w = q.w
            #print(t)
            br.sendTransform(t)
            #print("pubblicato")


def main():
    try:
        rospy.init_node('calibration')
        calibration()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



if __name__ == "__main__":
    main()