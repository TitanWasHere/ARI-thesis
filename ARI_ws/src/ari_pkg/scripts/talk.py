#!/usr/bin/env python
import rospy
import json
import speech_recognition as sr
from std_msgs.msg import String, Float64MultiArray
import os
#from pal_navigation_msgs.msg import GoToPOIActionGoal
from gtts import gTTS
import subprocess
from visualization_msgs.msg import InteractiveMarkerUpdate
# import .srv
from ari_pkg.srv import msgs, msgsResponse, wavs_msgs, wavs_msgsResponse

wavs_name_dir = "andre" # prima era andre
topics_file_name = "topics.json"
responses_file_name = "responses.json" # prima era responses.json



class SpeechRecognizer:
    def __init__(self):
        rospy.init_node('speech_recognizer')
        
        self.path_wavs = "../wavs/" + wavs_name_dir + "/"
        self.resp_cycle = {}

        # Carica il file JSON con i topic
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(self.current_dir, topics_file_name), 'r') as file:
            self.topics = json.load(file)
        
        self.result_from_move = String()
        # {
        #     "topic_name": {
        #         "fileName": ["name1", "name2", "name3],
        #         "text_if_error": "...text..."
        #     }
        # }
        # Carica il file JSON con le risposte
        with open(os.path.join(self.current_dir, responses_file_name), 'r') as file:
            self.responses = json.load(file)
            
            for topicName, data in self.responses.items():
                self.resp_cycle[topicName] = [len(data["fileName"]), 0]

        # Inizializza il recognizer di speech_recognition
        self.recognizer = sr.Recognizer()

        # Avvia l'ascolto del microfono
        self.listen_microphone()

        

    # def callback(self, data):
    #     print("[INFO]: Received: " + data.data)
    #     self.result_from_move.data = data.data

    def listen_microphone(self):
        print("[INFO]: In ascolto... Parla pure!")
        with sr.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source)
            audio = self.recognizer.listen(source)

        print("[INFO]: Riconoscimento in corso...")
        try:
            self.recognizer = sr.Recognizer()
            spoken_text = self.recognizer.recognize_google(audio, language='it-IT').lower()
            print("Hai detto: " + spoken_text)
            found_something = False
            for topic, keywords in self.topics.items():
                for keyword in keywords:
                    if keyword.lower() in spoken_text:
                        
                        if topic == "addio":
                            continue_listen = False
                            response = "addio"
                            #os.system("aplay " + os.path.join(self.current_dir,self.path_wavs) + response + ".wav")
                            self.say_something(response, response)
                            exit()
                        # If it's a normal spoken response
                        elif topic != "goto":
                            # Responses json structure:
                            # {
                            # "topic": {
                            #   "fileName": ["response1", "response2", "response3"],
                            #   "text_if_error": "text in case the file is not found"
                            #   }
                            # }


                            response = self.responses[topic]["fileName"]
                            response = response[self.resp_cycle[topic][1]]
                            self.resp_cycle[topic][1] = (self.resp_cycle[topic][1]+1)%(self.resp_cycle[topic][0])
                            print("[INFO]: Trovata parola chiave: %s, Risposta: %s", keyword, response)

                            self.say_something(response, response)
                        
                        else:
                            try:
                                rospy.wait_for_service("move_to_POI")
                                call = rospy.ServiceProxy("move_to_POI", msgs)
                                res = call(spoken_text).msg

                                if res == "not_found":
                                    response = "no_POI"
                                else:
                                    response = "found_POI"


                            except rospy.ROSException:
                                res = "error"
                                rospy.logerr("Timeout reached")
                                self.say_something("Errore, non ho trovato niente, prova a ripetere", "no_POI")
                                
                        found_something = True
                
                if found_something:
                    break

            print("[INFO]: continuo...")                    

            #print("[INFO]: Nessuna corrispondenza trovata.")
        except sr.UnknownValueError:
            rospy.logwarn("testo non riconosciuto")
        except sr.RequestError as e:
            rospy.logerr("errore durante la richiesta a Google")
        
        self.listen_microphone()
        
    def say_something(self, text, fileName=None):
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
        speech_recognizer = SpeechRecognizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

