#! /usr/bin/env python

import rospy
import tf2_ros
from pyquaternion import Quaternion
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from ArUco_data import *

class TransformPublisher:

    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.pub_transform = rospy.Publisher("/transform", TFMessage, queue_size=2)

        while not rospy.is_shutdown():
            self.publish_transform()


    def publish_transform(self):

        tfm = TFMessage()

        #print(msg)
        #print("transform_callback")
        trans_map_odom = self.get_transform("map", "odom")
        tfm.transforms.append(trans_map_odom)
        #print("after get_transform")
        #self.map_to_odom = self.assign_values(trans)
        

        trans_odom_base = self.get_transform("odom", "base_footprint")
        tfm.transforms.append(trans_odom_base)
        #self.odom_to_baseFootprint = self.assign_values(trans)
        

        trans_base_camera = self.get_transform("base_footprint", "torso_front_camera_color_optical_frame")
        tfm.transforms.append(trans_base_camera)
        #self.baseFootprint_to_camera = self.assign_values(trans)
        
        
        

        trans_map_base = self.get_transform("map", "base_footprint")
        tfm.transforms.append(trans_map_base)
        #self.map_to_baseFootprint = self.assign_values(trans)

        trans_odom_camera = self.get_transform("odom", "torso_front_camera_color_optical_frame")
        tfm.transforms.append(trans_odom_camera)
        #self.odom_to_camera = self.assign_values(trans)

        trans_map_camera = self.get_transform("map", "torso_front_camera_color_optical_frame")
        tfm.transforms.append(trans_map_camera)
        #self.map_to_camera = self.assign_values(trans)

        trans_map_cameraHead = self.get_transform("map", "head_front_camera_color_optical_frame")
        tfm.transforms.append(trans_map_cameraHead)
        #self.map_to_headCamera = self.assign_values(trans)

        i = 0
        trans_map_aruco = []
        for key in ArUcos:
            name = "aruco_" + key
            #print("pubblico " + name)
            trans_map_aruco.append(self.get_transform("map", name))
            #print(trans_map_aruco[i])
            #self.map_to_aruco[key] = self.assign_values(trans)
            tfm.transforms.append(trans_map_aruco[i])
            i+=1
            

        self.pub_transform.publish(tfm)
        
        






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
        




def main():
    try:
        rospy.init_node("transform_publisher")
        TransformPublisher()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()