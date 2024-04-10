#! /usr/bin/env python

# ROS node to move the robot to a Point of Interest (POI) using the MoveBaseAction
# The POI is received as a string from the /POI topic
# The robot moves to the POI and then stops

import rospy
from pal_navigation_msgs.msg import GoToPOIActionGoal
from std_msgs.msg import Header

class MoveToPOI:
    def __init__(self):
        # Initialize the node
        rospy.init_node('move_to_poi', anonymous=True)
        self.goal = rospy.Publisher('/poi_navigation_server/go_to_poi/goal', GoToPOIActionGoal, queue_size=2)

        

        self.move()

    def move(self):
        self.goal_msg = GoToPOIActionGoal()
        self.goal_msg.header.seq = 0
        self.goal_msg.header.stamp.secs = 0
        self.goal_msg.header.stamp.nsecs = 0
        self.goal_msg.header.frame_id = 'map'
        self.goal_msg.goal_id.stamp.secs = 0
        self.goal_msg.goal_id.stamp.nsecs = 0
        self.goal_msg.goal_id.id = ''
        self.goal_msg.goal.poi.data = 'ari_16c_dockstation'
        self.goal.publish(self.goal_msg)


        

if __name__ == '__main__':
    try:
        MoveToPOI()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass