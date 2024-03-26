#!/usr/bin/env python  
import roslib
roslib.load_manifest('detection')
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        # 2.85708618164 -1.28423094749 0.05 0.0 0.0 0.989162684798 0.146823645927 map aruco 100
        br.sendTransform((2.85708618164, -1.28423094749, 0.05),
                         (0.0, 0.0, 0.989162684798, 0.146823645927),
                         rospy.Time.now(),
                         "map",
                         "aruco")
        rate.sleep()
