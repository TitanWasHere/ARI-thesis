import rospy
import os
import json
#from gtts import gTTS
import subprocess 

from ari_pkg.srv import msgs



class download:
    def __init__(self):
        rospy.init_node("download_wavs")
        
        message = rospy.Service("callback", msgs, self.callback)
        print("[" + message + "]: dopo message")

        rospy.spin()

    def callback(self, req):
        print("Received: " + req.text)

        return req.text
        




if __name__ == "__main__":
    try:
        download()
        
    except rospy.ROSInterruptException:
        pass