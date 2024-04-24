#!/usr/bin/env python
import rospy
import os
from collections import deque
import json
import speech_recognition as sr
from std_msgs.msg import String, Float64MultiArray
from gtts import gTTS
import subprocess 
from pal_navigation_msgs.msg import GoToPOIActionGoal
from visualization_msgs.msg import InteractiveMarkerUpdate

class checkMovement:
    def __init__(self):
        rospy.init_node('check_POI')
        self.POIs = rospy.Subscriber('/poi_marker_server/update', InteractiveMarkerUpdate, self.POI_callback)
        self.goal = rospy.Publisher('/poi_navigation_server/go_to_poi/goal', GoToPOIActionGoal, queue_size=2)
        self.sub_speech = rospy.Subscriber('/POI/move/check', String, self.callback)
        self.pub_status = rospy.Publisher('/POI/move/status', String, queue_size=2)

        self.allMarkers = {}

        self.recognizer = sr.Recognizer()

    def POI_callback(self, data):
        if data.markers == []:
            return
        
        for marker in data.markers:
            self.allMarkers[marker.name] = marker.pose.position


    def callback(self, data):
        print("[INFO]: Received: " + data.data)
        spoken_text = data.data

        # Check if the spoken text contains a POI (in the json)
        current_dir = os.path.dirname(os.path.abspath(__file__))
        # Json structure:
        # {
        #     "POI_name": {
        #       "wav_name": name,  
        #       "spoken_name": nome a voce,
        #       "keywords": ["keyword1", "keyword2", "keyword3"]
        #     }
        # }
        with open(os.path.join(current_dir, 'points_of_interest.json'), 'r') as file:
            self.poi = json.load(file)

        poi_name = None
        print("[INFO]: poi detected in map: " + str(self.poi))
        detected_POI = []
        found = False
        # for name, poilist in self.poi.items():
        #     found = False
        #     for keyword in poilist:
        #         if not found and keyword.lower() in spoken_text.lower():
        #             poi_name = name
        #             found = True
        #             detected_POI.append(poi_name)

        for name, poilist in self.poi.items():
            found = False
            for keyword in poilist["keywords"]:
                if not found and keyword.lower() in spoken_text.lower():
                    poi_name = name
                    found = True
                    detected_POI.append(poi_name)

        response = False

        if len(detected_POI) == 0:
            rospy.logwarn("No POI found")
            self.pub_status.publish("not_found")
            return
        else:
            print("[INFO]: POI found: " + str(detected_POI))
            response = self.confirm_POI(detected_POI)

        if response is False:
            self.pub_status.publish("not_found")
        else:
            self.goto_POI(response)

        

        # if self.check_POI(poi_name) is not None:
        #     print("[INFO]: ["+poi_name+"] found")
        #     self.gotoPOI(poi_name)
        # else:
        #     print("[INFO]: ["+poi_name+"] not found")

    def confirm_POI(self, POIs):
        print("[INFO]: Point to check: " + str(POIs[0]))
        self.say_something("Confermi di voler andare a " + POIs[0] + "? Dimmi si o no")
        response = self.wait_confirm()
        if response is False and len(POIs) > 1:
            # In caso non va bene, salvo in una stringa tutti gli altri POI trovati
            del POIs[0]
            others = ""
            for poi in POIs:
                others += poi + " "

            self.say_something("Dimmi se intedevi uno dei seguenti punti, altrimenti dimmi nessuno: " + others)

            return self.wait_confirm_for_more_POIs(POIs)
        else:
            return POIs[0]
    
    def goto_POI(self, name):

        self.say_something("Sto andando a " + name)
        
        self.goal_msg = GoToPOIActionGoal()
        self.goal_msg.header.seq = 0
        self.goal_msg.header.stamp = rospy.Time.now()
        self.goal_msg.header.frame_id = 'map'
        self.goal_msg.goal_id.stamp = rospy.Time.now()
        self.goal_msg.goal_id.id = ''
        self.goal_msg.goal.poi.data = name
        self.goal.publish(self.goal_msg)
                        

    def wait_confirm_for_more_POIs(self, POIs):
        while True:
            with sr.Microphone() as source:
                self.recognizer.adjust_for_ambient_noise(source)
                audio = self.recognizer.listen(source)

            spoken_text = self.recognizer.recognize_google(audio, language='it-IT').lower()
            print("Hai detto: " + spoken_text)
            if spoken_text != "":
                if "nessuno" in spoken_text.lower():
                    return False
                
                for poi in POIs:
                    if poi.lower() in spoken_text:
                        return poi
                

    def wait_confirm(self):
        while True:
            # Parla al microfono e ritorna True se dice "si"
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
            

    # The text should anyway be written because if for any reason the wav file is not found, the robot will say the text anyway
    def say_something(self, text, fileName=None):
        if fileName is None or self.checkExistance(fileName) is False :
            tts = gTTS(text=text, lang='it')
            tts.save("response.mp3")
                
            subprocess.call(['ffmpeg', '-i', "response.mp3", "response.wav"])
                
            os.system("aplay response.wav")
            os.system("rm response.wav")
            os.system("rm response.mp3")


    def checkExistance(self, fileName):
        name = os.path.join(self.current_dir,self.path_wavs) + fileName + ".wav"
        if os.path.exists(name):
            os.system("aplay " + name)
            return True
        else:
            return False
        
if __name__ == '__main__':
    try:
        checkMovement()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass