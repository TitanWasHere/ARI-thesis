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
    
    trasformation = tfBuffer.lookup_transform("odom", "torso_front_camera_color_frame", rospy.Time(), rospy.Duration(1.0))
    #trasformation = tfBuffer.lookup_transform("base_footprint", "base_link", rospy.Time(), rospy.Duration(1.0))
    print(trasformation) 

    pos_x = trasformation.transform.translation.x
    pos_y = trasformation.transform.translation.y
    pos_z = trasformation.transform.translation.z

    quat_x = trasformation.transform.rotation.x
    quat_y = trasformation.transform.rotation.y
    quat_z = trasformation.transform.rotation.z
    quat_w = trasformation.transform.rotation.w
            
    
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass