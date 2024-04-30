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


rospy.init_node('tf_listener')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

while not rospy.is_shutdown():
    try:
        print("prima del lookup")
        trans = tfBuffer.lookup_transform('map', 'torso_front_camera_color_frame', rospy.Time())
        print(trans)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        continue


rospy.spin()