import rospy
from std_msgs.msg import String
import tf2_ros
import geometry_msgs.msg
from pyquaternion import Quaternion



def main():
    rospy.init_node('matrix', anonymous=True)

    # get position and orientation from 2 frames with tf2
    tfBuffer = tf2_ros.Buffer(rospy.Duration(10))
    listener = tf2_ros.TransformListener(tfBuffer)
    
    trasformation = tfBuffer.lookup_transform("base_footprint", "torso_front_camera_color_frame", rospy.Time(), rospy.Duration(1.0))
    #trasformation = tfBuffer.lookup_transform("base_footprint", "base_link", rospy.Time(), rospy.Duration(1.0))
    print(trasformation) 
            
    
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass