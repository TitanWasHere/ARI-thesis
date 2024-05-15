#!/usr/bin/env python
import rospy
import os
from collections import deque
import time
import json
import speech_recognition as sr
from std_msgs.msg import String, Float64MultiArray
from gtts import gTTS
import subprocess 
from geometry_msgs.msg import Pose
from pal_navigation_msgs.msg import GoToPOIActionGoal
from visualization_msgs.msg import InteractiveMarkerUpdate
from ari_pkg.srv import msgs, msgsResponse, msgPOI, msgPOIResponse, wavs_msgs, wavs_msgsResponse


class confirm_move:
    def __init__(self):
        rospy.init_node('check_POI')

        # Service which waits for the confirmation of the movement
        s = rospy.Service("confirm_move" , msgs, self.callback)
        
        self.current_dir = os.path.dirname(os.path.abspath(__file__))

        self.recognizer = sr.Recognizer()

        rospy.spin()
    
    
    def callback(self, data):
        # string[] poi_names
        # ---
        # bool found
        # string poi_name_out

        if len(data.poi_names) == 0:
            return msgPOIResponse(False, "")
        else:
            res = self.confirm_POI(data.poi_names)
            if res is False:
                return msgPOIResponse(False, "")
            else:
                return msgPOIResponse(True, res)
            


    
    def confirm_POI(self, POIs):
        print("[INFO]: Point to check: " + str(POIs[0]))
        # TODO : CHECK THE CONFIRM
        self.say_something("Confermi di voler andare a " + POIs[0] + "? Dimmi si o no", "confirm_"+POIs[0])
        response = self.wait_confirm()

        if response is False:
            if len(POIs) > 1:
                # In caso non va bene, salvo in una stringa tutti gli altri POI trovati
                del POIs[self.firstFound]
                others = ""
                for _ , poi_data in POIs.items():
                    others += poi_data["spoken_name"] + ", "

                self.say_something("Dimmi se intedevi uno dei seguenti punti, altrimenti dimmi nessuno: " + others)

                return self.wait_confirm_for_more_POIs(POIs)
            else:
                return False
        else:
            return self.POIs[0]
         

    def wait_confirm_for_more_POIs(self, POIs):
        count = 0
        while True:
            self.recognizer = sr.Recognizer()
            
            with sr.Microphone() as source:
                self.recognizer.adjust_for_ambient_noise(source)
                audio = self.recognizer.listen(source)

            spoken_text = self.recognizer.recognize_google(audio, language='it-IT').lower()
            print("Hai detto: " + spoken_text)
            if spoken_text != "":
                if "nessuno" in spoken_text:
                    return False
                
                for poi in POIs.items():
                    if poi.lower() in spoken_text:
                        return poi
                
                if count != 5:
                    count += 1
                    self.say_something("Non ho capito. Ripeti per favore", "repeat")
                else:
                    return False
    
    def wait_confirm(self):
        while True:
            self.recognizer = sr.Recognizer()
            with sr.Microphone() as source:
                self.recognizer.adjust_for_ambient_noise(source)
                audio = self.recognizer.listen(source)

            spoken_text = self.recognizer.recognize_google(audio, language='it-IT').lower()
            print("Hai detto: " + spoken_text)
            if spoken_text != "":
                if "si" in spoken_text or "confermo" == spoken_text.lower():
                    return True
                else:
                    return False

    def say_something(self, text, fileName = None):
        name = os.path.join(self.current_dir,self.path_wavs) + fileName + ".wav"
        if os.path.isfile(name) is False:
            
            # call the service wav_creator
            rospy.wait_for_service("wav_creator")
            call = rospy.ServiceProxy("wav_creator", wavs_msgs)
            resp = call(text, fileName)
            if resp.msg == "error":
                rospy.logerr("Error while creating the wav file")
                return
        
        os.system("aplay " + name + ".wav")


if __name__ == '__main__':
    try:
        confirm_move()
    except rospy.ROSInterruptException:
        pass
