import rospy
import json
from std_msgs.msg import *
from pal_navigation_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from visualization_msgs.msg import InteractiveMarkerUpdate

class PrintPOIs:
    def __init__(self):
        rospy.init_node('print_pois')
        self.POIs = rospy.Subscriber('/poi_marker_server/update', InteractiveMarkerUpdate, self.POI_callback)
        
    def POI_callback(self, data):
        print(data)
    

if __name__ == '__main__':
    try:
        PrintPOIs()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass