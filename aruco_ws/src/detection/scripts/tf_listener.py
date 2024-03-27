#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
import tf2_ros

class TfListener:
    def __init__(self):
        rospy.init_node('tf_listener', anonymous=True)
        
        # Buffer that stores several seconds of transforms for easy lookup by the listener
        self.tf_buffer = tf2_ros.Buffer()
        
        # Listener for the broadcast transform message
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Pose in source frame (`sensor_link`)
        self.pose_in = PoseStamped()
        
        # Pose in target frame (`arm_end_link`)
        self.pose_out = PoseStamped()
        
        # Subscription to pose published by sensor node
        self.pose_sub = rospy.Subscriber('/detected_object', PoseStamped, self.pose_callback)

    def pose_callback(self, msg):
        try:
            # Store the incoming pose in pose_in_
            self.pose_in = msg
            
            # Transforms the pose between the source frame and target frame
            self.pose_out = self.tf_buffer.transform(self.pose_in, 'arm_end_link', rospy.Duration(1))
            
            # Log coordinates of pose in target frame
            rospy.loginfo("Object pose in 'arm_end_link' is:\n x,y,z = %.1f,%.1f,%.1f",
                          self.pose_out.pose.position.x,
                          self.pose_out.pose.position.y,
                          self.pose_out.pose.position.z)
        except (tf2_ros.TransformException, tf2_geometry_msgs.TransformException) as ex:
            rospy.logwarn("Could not find object position in 'arm_end_link' frame.")
            return

if __name__ == '__main__':
    try:
        tf_listener = TfListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass