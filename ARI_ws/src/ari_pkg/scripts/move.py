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
from pal_navigation_msgs.msg import GoToPOIActionGoal
from visualization_msgs.msg import InteractiveMarkerUpdate
from ari_pkg.srv import msgs, msgsResponse, msgPOI, msgPOIResponse, wavs_msgs, wavs_msgsResponse

wavs_name_dir = "muse"
poi_file_name = "muse_poi.json"


class move:
    def __init__(self):

        # No more subscribers and publishers, only the service server
        rospy.init_node('check_POI')

        self.POIs = rospy.Subscriber('/poi_marker_server/update', InteractiveMarkerUpdate, self.POI_callback)
        self.goal = rospy.Publisher('/poi_navigation_server/go_to_poi/goal', GoToPOIActionGoal, queue_size=2)
        
        s = rospy.Service("move_to_POI" , msgs, self.callback)

        self.firstFound = ""
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.path_wavs = "../wavs/" + wavs_name_dir + "/"
        self.allMarkers = {}
        self.detected_POI = {}
        
        self.recognizer = sr.Recognizer()

    def POI_callback(self, data):
        if data.markers == []:
            return
        
        for marker in data.markers:
            self.allMarkers[marker.name] = marker.pose.position

    def callback(self, data):
        
        spoken_text = data.msg
        print("[INFO]: Received: " + spoken_text)
        # Json structure:
        # {
        #     "POI_name": {
        #       "wav_name": name,  
        #       "spoken_name": nome a voce,
        #       "keywords": ["keyword1", "keyword2", "keyword3"]
        #     }
        # }
        with open(os.path.join(self.current_dir, poi_file_name), 'r') as file:
            self.poi = json.load(file)

        poi_name = None
        found = False

        for name, poilist in self.poi.items():
            found = False
            for keyword in poilist["keywords"]:
                if not found and keyword.lower() in spoken_text.lower():
                    poi_name = name
                    found = True
                    self.detected_POI[poi_name] = self.poi[poi_name]["spoken_name"]
                    if self.firstFound == "":
                        self.firstFound = poi_name

        
        response = False
        if len(self.detected_POI) == 0:
            rospy.logwarn("No POI found")
            stat = String()
            
            return msgsResponse("not_found")
        else:
            print("[INFO]: POI found: " + str(self.detected_POI))
            
            # Call the service to check the POI called "confirm_move"
            rospy.wait_for_service("confirm_move")
            call = rospy.ServiceProxy("confirm_move", msgPOI)

            arr_of_POIs = [self.firstFound]
            del self.detected_POI[self.firstFound]
            for name, data in self.detected_POI.items():
                arr_of_POIs.append(name)
            
            resp = call(arr_of_POIs)
            response = resp.found

        if response is False:
            stat = String()
            stat.data = "not_found"
            return msgsResponse("not_found")
        else:
            self.goto_POI(response.poi_name_out)
            return msgsResponse("found")

        
    def goto_POI(self, name):
        sp_name = self.detected_POI[name]

        self.say_something("Vado a " + sp_name , "goto_" + sp_name)
        
        self.goal_msg = GoToPOIActionGoal()
        self.goal_msg.header.seq = 0
        self.goal_msg.header.stamp = rospy.Time.now()
        self.goal_msg.header.frame_id = 'map'
        self.goal_msg.goal_id.stamp = rospy.Time.now()
        self.goal_msg.goal_id.id = ''
        self.goal_msg.goal.poi.data = name
        self.goal.publish(self.goal_msg)
        print("[INFO]: vado alla pos " + sp_name + " con nome " + name)

                
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
        move()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass