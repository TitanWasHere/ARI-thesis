#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

class Sensor:
    def __init__(self):
        rospy.init_node('sensor', anonymous=True)
        
        # Timer for the simulated detected object
        self.object_timer = rospy.Timer(rospy.Duration(1), self.object_callback)
        
        # Publisher for the simulated detected object
        self.pose_pub = rospy.Publisher('/detected_object', PoseStamped, queue_size=10)
        
        # Pose to publish
        self.pose = PoseStamped()
        
        # Aux variable that determines the detected object’s position
        self.count = 0

    def object_callback(self, event):
        # All transforms must be correctly time-stamped
        self.pose.header.stamp = rospy.Time.now()
        self.pose.header.frame_id = "base_link"
        
        if self.count % 2:
            self.pose.pose.position.x = 1.0
            self.pose.pose.position.y = 1.0
        else:
            self.pose.pose.position.x = 2.0
            self.pose.pose.position.y = 3.0
        
        # Change the detected object’s position, depending on whether count_ is even or odd
        self.count += 1
        self.pose_pub.publish(self.pose)

if __name__ == '__main__':
    try:
        sensor = Sensor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
