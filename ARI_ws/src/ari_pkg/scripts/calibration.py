#! /usr/bin/env python

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
from collections import namedtuple
import geometry_msgs.msg
from tf2_msgs.msg import TFMessage

# rospy.init_node('tf_listener')
# tfBuffer = tf2_ros.Buffer()
# listener = tf2_ros.TransformListener(tfBuffer)

# while not rospy.is_shutdown():
#     try:
#         print("prima del lookup")
#         trans = tfBuffer.lookup_transform('map', 'torso_front_camera_color_frame', rospy.Time())
#         print(trans)
#     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#         continue
INTRINSIC_CAMERA_REALSENSE_1280 = np.array([[fx_RealSense_1280, 0, cx_RealSense_1280], [0, fy_RealSense_1280, cy_RealSense_1280], [0, 0, 1]])
DISTORTION_CAMERA_REALSENSE_1280 = np.array([k1_RealSense_1280, k2_RealSense_1280, p1_RealSense_1280, p2_RealSense_1280, k3_RealSense_1280])


class calibration:


    def __init__(self):
        self.pub_torso = rospy.Publisher('/torso_front_camera/color/aruco', Image, queue_size=2)
        self.pub_front = rospy.Publisher('/head_front_camera/color/aruco', Image, queue_size=2)
        self.pub_myPos = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=2)
        self.sub_torso = rospy.Subscriber('/torso_front_camera/color/image_raw', Image, self.image_callback, queue_size=2)
        self.sub_front = rospy.Subscriber('/head_front_camera/color/image_raw/compressed', CompressedImage, self.image_compressed_callback, queue_size=2)
        #self.listener = tf.TransformListener()
        #self.sub_tf = rospy.Subscriber('/tf', TFMessage, self.transform_callback)

        

        self.bridge = CvBridge()
        self.received_image = False

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.map_to_odom = None
        self.odom_to_baseFootprint = None
        self.baseFootprint_to_camera = None
        self.map_to_aruco = None
        self.map_to_baseFootprint = None
        self.odom_to_camera = None
        self.map_to_camera = None

        
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

        

        print(msg)
        print("transform_callback")
        trans = self.get_transform("map", "odom")
        print("after get_transform")
        self.map_to_odom = self.assign_values(trans)
        

        trans = self.get_transform("odom", "base_footprint")
        self.odom_to_baseFootprint = self.assign_values(trans)
        

        trans = self.get_transform("base_footprint", "torso_front_camera_color_optical_frame")
        self.baseFootprint_to_camera = self.assign_values(trans)

        trans = self.get_transform("map", "aruco")
        self.map_to_aruco = self.assign_values(trans)

        trans = self.get_transform("map", "base_footprint")
        self.map_to_baseFootprint = self.assign_values(trans)

        trans = self.get_transform("odom", "torso_front_camera_color_optical_frame")
        self.odom_to_camera = self.assign_values(trans)

        trans = self.get_transform("map", "torso_front_camera_color_optical_frame")
        self.map_to_camera = self.assign_values(trans)


        # self.print_values("map", "odom", self.map_to_odom)
        # self.print_values("odom", "base_footprint", self.odom_to_baseFootprint)
        # self.print_values("base_footprint", "torso_front_camera_color_frame", self.baseFootprint_to_camera)
        # self.print_values("map", "aruco", self.map_to_aruco)
        # self.print_values("map", "base_footprint", self.map_to_baseFootprint)
        # self.print_values("odom", "torso_front_camera_color_frame", self.odom_to_camera)
        #self.print_values("map", "torso_front_camera_color_frame", self.map_to_camera)

        # print("^^^^^^^^^^^^^^^")






    def assign_values(self, values):
        var = namedtuple("Pose", ["position", "orientation"])
        position = namedtuple("Position", ["x", "y", "z"])
        orientation = namedtuple("Orientation", ["x", "y", "z", "w"])
        pos = position(values.transform.translation.x, values.transform.translation.y, values.transform.translation.z)
        ori = orientation(values.transform.rotation.x, values.transform.rotation.y, values.transform.rotation.z, values.transform.rotation.w)
        return var(pos, ori)
        
    def get_transform(self, frame_x, frame_y, ok = False):
        

        trans = None
        while not rospy.is_shutdown() and not ok:
            try:
                #print("Prima di " + frame_x + " su " + frame_y)
                trans = self.tfBuffer.lookup_transform(frame_x, frame_y, rospy.Time())
                #print(trans)
                ok = True    
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
        
        return trans

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
                #print('[' + str(ids[i]) + ']: { tvec: ' + str(tvec[i][0]) + ' rvec: ' + str(rvec[i][0]) + ' }')

            #frame = cv2.aruco.drawAxis(frame, INTRINSIC_CAMERA, DISTORTION_CAMERA, rvec, tvec, 0.1)
            return frame, tvec, rvec, ids
        return frame, None, None, None
    
    def manage_frame(self, frame, tvec, rvec, ids, type):

        self.transform_callback(None)
        print("after transform_callback ")
        if ids is not None:
            for i in range(len(ids)):
                try:
                    # aruco = ArUcos[str(ids[i])]
                    # sleep(2)
                    
                    while self.map_to_odom is None or self.odom_to_baseFootprint is None or self.baseFootprint_to_camera is None or self.map_to_aruco is None :
                        print('Waiting for tf...')
                        print(self.map_to_odom)
                        print(self.odom_to_baseFootprint)
                        print(self.baseFootprint_to_camera)
                        print(self.map_to_baseFootprint)
                        print(self.map_to_aruco)
                        sleep(1)
                    

                    map_camera_ori_quat = Quaternion(a = self.map_to_camera.orientation.w, b = self.map_to_camera.orientation.x, c = self.map_to_camera.orientation.y, d = self.map_to_camera.orientation.z)
                    map_T_camera = np.zeros((4,4))
                    map_T_camera[0:3, 0:3] = map_camera_ori_quat.rotation_matrix
                    map_T_camera[0:3, 3] = np.array([self.map_to_camera.position.x, self.map_to_camera.position.y, self.map_to_camera.position.z])
                    map_T_camera[3, 3] = 1

                    camera_T_aruco = np.zeros((4,4))
                    camera_T_aruco[0:3, 0:3] = cv2.Rodrigues(rvec[i])[0]
                    camera_T_aruco[0:3, 3] = tvec[i][0]
                    camera_T_aruco[3, 3] = 1
                    
                    map_T_aruco = np.dot(map_T_camera, camera_T_aruco)

                    self.publish_tf2("map", "new_aruco", map_T_aruco)


                    # # T1 * T2 = map_T_camera * camera_T_aruco
                    # # T1 * T2 ^ -1 = map_T_camera * camera_T_aruco ^ -1
                    # # T2 * T1 = camera_T_aruco * map_T_camera
                    # # T2 ^ -1 * T1 = camera_T_aruco ^ -1 * map_T_camera
                    # self.publish_tf2("map", "T1xT2_inv", np.dot(map_T_camera, np.linalg.inv(camera_T_aruco)))
                    # self.publish_tf2("map", "T2xT1", np.dot(camera_T_aruco, map_T_camera))
                    # self.publish_tf2("map", "T2_invxT1", np.dot(np.linalg.inv(camera_T_aruco), map_T_camera))

                    odom_camera_ori_quat = Quaternion(a = self.odom_to_camera.orientation.w, b = self.odom_to_camera.orientation.x, c = self.odom_to_camera.orientation.y, d = self.odom_to_camera.orientation.z)
                    odom_T_camera = np.zeros((4,4))
                    odom_T_camera[0:3, 0:3] = odom_camera_ori_quat.rotation_matrix
                    odom_T_camera[0:3, 3] = np.array([self.odom_to_camera.position.x, self.odom_to_camera.position.y, self.odom_to_camera.position.z])
                    odom_T_camera[3, 3] = 1

                    odom_T_aruco = np.dot(odom_T_camera, camera_T_aruco)

                    self.publish_tf2("odom", "odom->cam->aruco", odom_T_aruco)

                    print("after while")

                    


                except KeyError:
                    print('Aruco not found')
                    return


        if type == "torso":
            self.pub_torso.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        elif type == "front":
            self.pub_front.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))


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
            br.sendTransform(t)


def main():
    try:
        rospy.init_node('calibration')
        calibration()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



if __name__ == "__main__":
    main()