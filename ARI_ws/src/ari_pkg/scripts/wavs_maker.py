#!/usr/bin/env python
import rospy
import json
import os
#from pal_navigation_msgs.msg import GoToPOIActionGoal
from gtts import gTTS
import subprocess
from visualization_msgs.msg import InteractiveMarkerUpdate
# import .srv
from ari_pkg.srv import msgs, msgsResponse, wavs_msgs, wavs_msgsResponse

wavs_name_dir = "andre" # prima era andre

class create_wavs:
    def __init__(self):
        rospy.init_node("create_wavs")

        # Create a service server
        self.message = rospy.Service("wav_creator", wavs_msgs, self.callback)

        rospy.spin()

    def callback(self, req):
        try:
            tts = gTTS(req.text, lang='it')    

            mp3name = "../wavs/" + wavs_name_dir + "/" + req.name + ".mp3"
            wavname = "../wavs/" + wavs_name_dir + "/" + req.name + ".wav"
            tts.save(mp3name)

            subprocess.call(['ffmpeg', '-i', mp3name, wavname])
            return wavs_msgsResponse("ok")
        except:
            return wavs_msgsResponse("error")
        


if __name__ == "__main__":
    try:
        create_wavs()
        
    except rospy.ROSInterruptException:
        pass