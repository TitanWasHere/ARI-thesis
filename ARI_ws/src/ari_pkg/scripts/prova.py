import rospy

from ari_pkg.srv import msgs


class prova: 
    def __init__(self):
        rospy.init_node("prova")
        
        rospy.wait_for_service("callback")
        self.callback = rospy.ServiceProxy("callback", msgs)
    
        resp = self.callback("ciao")
        
        
        print(resp.msg)

        rospy.spin()


if __name__ == "__main__":
    try:
        prova()
        
    except rospy.ROSInterruptException:
        pass