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
        
        self.sub_speech = rospy.Subscriber('/POI/move/check', String, self.callback)
        self.pub_status = rospy.Publisher('/POI/move/status', String, queue_size=2)

        self.allMarkers = {}

    def POI_callback(self, data):
        if data.markers == []:
            return
        
        for marker in data.markers:
            self.allMarkers[marker.name] = marker.pose.position


    def callback(self, data):
        rospy.loginfo("Received: " + data.data)
        spoken_text = data.data

        # Check if the spoken text contains a POI (in the json)
        with open('points_of_interest.json', 'r') as file:
            self.poi = json.load(file)

        poi_name = None
        rospy.loginfo("poi detected in map: " + str(self.poi))
        detected_POI = []
        found = False
        for name, poilist in self.poi.items():
            found = False
            for keyword in poilist and found is False:
                if keyword.lower() in spoken_text.lower():
                    poi_name = name
                    found = True
                    detected_POI.append(poi_name)

        response = False

        if len(detected_POI) == 0:
            rospy.logwarn("No POI not found")
            self.pub_status.publish("not_found")
            return
        else:
            rospy.loginfo("POI found: " + str(detected_POI))
            response = self.confirm_POI(detected_POI)

        if response is False:
            self.pub_status.publish("not_found")
        else:
            self.goto_POI(response)

        

        # if self.check_POI(poi_name) is not None:
        #     rospy.loginfo("["+poi_name+"] found")
        #     self.gotoPOI(poi_name)
        # else:
        #     rospy.loginfo("["+poi_name+"] not found")

    def confirm_POI(self, POIs):
        rospy.loginfo("Point to check: " + str(POIs[0]))
        tts = gTTS(text="Confermi di voler andare a " + POIs[0] + "? Dimmi si o no", lang='it')
        tts.save("confirm.wav")
        os.system("aplay confirm.wav")

        # Se si bugga allora metti un wait che finisca il microfono di parlare
        os.system("rm confirm.wav")
        response = self.wait_confirm()
        if response is False and len(POIs) > 1:
            # In caso non va bene, salvo in una stringa tutti gli altri POI trovati
            del POIs[0]
            others = ""
            for poi in POIs:
                others += poi + " "

            tts = gTTS(text="Dimmi se intedevi uno dei seguenti punti, altrimenti dimmi nessuno: " + others, lang='it')
            tts.save("confirm.wav")
            os.system("aplay confirm.wav")
            os.system("rm confirm.wav")

            return self.wait_confirm_for_more_POIs(POIs)
        else:
            return POIs[0]
    
        
        # if response is False:
        #     self.pub_status.publish("not_found")
        # else:
        #     if self.check_POI(response) is not None:
        #         rospy.loginfo("["+response+"] found")
        #         self.gotoPOI(response)
        #     else:
        #         rospy.loginfo("["+response+"] not found")
        #         self.pub_status.publish("not_found")

        #         tts = gTTS(text="Mi dispiace, il punto non esiste nella mappa", lang='it')
        #         tts.save("not_found.wav")
        #         os.system("aplay not_found.wav")
        #         os.system("rm not_found.wav")


    def goto_POI(self, name):
        tts = gTTS(text="Sto andando a " + name, lang='it')
        tts.save("going.wav")
        os.system("aplay going.wav")
        os.system("rm going.wav")
        self.goal_msg = GoToPOIActionGoal()
        self.goal_msg.header.seq = 0
        self.goal_msg.header.stamp = rospy.Time.now()
        self.goal_msg.header.frame_id = 'map'
        self.goal_msg.goal_id.stamp = rospy.Time.now()
        self.goal_msg.goal_id.id = ''
        self.goal_msg.goal.poi.data = name
        self.goal.publish(self.goal_msg)
                        

    def wait_confirm_for_more_POIs(self, POIs):
        with sr.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source)
            audio = self.recognizer.listen(source)

        spoken_text = self.recognizer.recognize_google(audio, language='it-IT').lower()
        print("Hai detto: " + spoken_text)
        while spoken_text != "":
            if "nessuno" in spoken_text.lower():
                return False
            
            for poi in POIs:
                if poi.lower() in spoken_text:
                    return poi
                

    def wait_confirm(self):
        # Parla al microfono e ritorna True se dice "si"
        with sr.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source)
            audio = self.recognizer.listen(source)

        spoken_text = self.recognizer.recognize_google(audio, language='it-IT').lower()
        print("Hai detto: " + spoken_text)
        while spoken_text != "":
            if "si" in spoken_text or "confermo" == spoken_text.lower():
                return True
            else:
                return False
        
if __name__ == '__main__':
    try:
        checkMovement()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass