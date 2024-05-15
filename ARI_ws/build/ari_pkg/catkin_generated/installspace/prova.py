import rospy

from ari_pkg.srv import msgs


class prova: 
    def __init__(self):
        rospy.init_node("prova")
        
        rospy.wait_for_service("callback")
        self.callback = rospy.ServiceProxy("callback", msgs)
        resp = self.callback("ciao")

        print(resp)

        rospy.spin()