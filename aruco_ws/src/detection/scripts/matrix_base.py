#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import TransformStamped

def transform_callback(msg):
    try:
        now = rospy.Time.now()
        listener.waitForTransform("odom", "map", now, rospy.Duration(1.0))
        (trans, rot) = listener.lookupTransform("odom", "map", now)
        print("Transform from map to odom:")
        print("Translation: ", trans)
        print("Rotation: ", rot)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn("Unable to lookup transform from frame_x to frame_y: %s", e)

if __name__ == '__main__':
    rospy.init_node('transform_listener')

    listener = tf.TransformListener()

    #rospy.Subscriber("/tf", TransformStamped, transform_callback)
    transform_callback(None)
    rospy.spin()
