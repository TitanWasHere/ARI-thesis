#! /usr/bin/env python

import tf2_ros
import rospy
from pyquaternion import Quaternion
import geometry_msgs.msg
from ArUco_data import *


class FramePublisher:
    def __init__(self):
        self.publish_aruco()

    def publish_aruco(self):
        
        while not rospy.is_shutdown():
            for key in ArUcos:
                aruco = ArUcos[key]

                q = Quaternion(a = aruco["orientation"][3], b = aruco["orientation"][0], c = aruco["orientation"][1], d = aruco["orientation"][2])


                name = "aruco_" + key
                #print("pubblico " + name)


                br = tf2_ros.TransformBroadcaster()
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "map"
                t.child_frame_id = name
                t.transform.translation.x = aruco["position"][0]
                t.transform.translation.y = aruco["position"][1]
                t.transform.translation.z = aruco["position"][2]

                t.transform.rotation.x = q.x
                t.transform.rotation.y = q.y
                t.transform.rotation.z = q.z
                t.transform.rotation.w = q.w
                #print(t)
                br.sendTransform(t)
                #print("Published " + name)
                



def main():
    try:
        rospy.init_node("frame_publisher")
        FramePublisher()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
