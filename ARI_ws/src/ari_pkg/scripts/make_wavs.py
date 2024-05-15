import rospy
import os
import json
#from gtts import gTTS
import subprocess 
import time

from ari_pkg.srv import msgs, msgsResponse



class download:
    def __init__(self):
        rospy.init_node("download_wavs")
        
        message = rospy.Service("callback", msgs, self.callback)
        print("dopo message")

        rospy.spin()

    def callback(self, req):
        print("Received: " + req.msg)
        print("Sending: zio_can")
        time.sleep(3)
        return msgsResponse("zio_can")
        




if __name__ == "__main__":
    try:
        download()
        
    except rospy.ROSInterruptException:
        pass