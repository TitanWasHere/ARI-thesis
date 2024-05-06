import rospy
from geometry_msgs.msg import Twist


class stop:
    def __init__(self):
        self.pub_velocity = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=2)
        #self.sub_velocity = rospy.Subscriber('/mobile_base_controller/cmd_vel', Twist, self.callback)
        

        
        
        self.count = 0


    def callback(self, msg):
        print("[INFO]: " + str(self.count))
        if self.count == 20:
            for i in range(50):
                print("STOOPPPPP")
                vel = Twist()
                vel.linear.x = 0
                vel.linear.y = 0
                vel.linear.z = 0
                vel.angular.x = 0
                vel.angular.y = 0
                vel.angular.z = 0

                self.pub_velocity.publish(vel)


            self.pub_velocity.publish(vel)

        if self.count == 50:
            # shutdown node
            rospy.signal_shutdown("Stop robot node")

        self.count += 1


if __name__ == "__main__":
    rospy.init_node('stop')
    stop()
    rospy.spin()
    print("[INFO]: Stop robot node started.")
