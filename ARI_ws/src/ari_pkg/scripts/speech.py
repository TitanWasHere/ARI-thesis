#!/usr/bin/env python
import rospy
import json
import speech_recognition as sr
from std_msgs.msg import String, Float64MultiArray
import os
from pal_navigation_msgs.msg import GoToPOIActionGoal
from gtts import gTTS
import subprocess
from visualization_msgs.msg import InteractiveMarkerUpdate

class SpeechRecognizer:
    def __init__(self):
        rospy.init_node('speech_recognizer')
        self.goal = rospy.Publisher('/poi_navigation_server/go_to_poi/goal', GoToPOIActionGoal, queue_size=2)
        self.check_goto = rospy.Publisher('/POI/move/check', String, queue_size=2)
        self.path_wavs = "../wavs/andre/"
        self.resp_cycle = {}

        current_dir = os.path.dirname(os.path.abspath(__file__))
        file_path = os.path.join(current_dir, 'nome_file.json')


        # Carica il file JSON con i topic
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(self.current_dir,'topics.json'), 'r') as file:
            self.topics = json.load(file)
            
        # Carica il file JSON con le risposte
        with open(os.path.join(self.current_dir,'responses.json'), 'r') as file:
            self.responses = json.load(file)
            
            for topicName, data in self.responses.items():
                self.resp_cycle[topicName] = [len(data["fileName"]), 0]

        # Inizializza il recognizer di speech_recognition
        self.recognizer = sr.Recognizer()

        # Avvia l'ascolto del microfono
        self.listen_microphone()



    def listen_microphone(self):
        print("[INFO]: In ascolto... Parla pure!")
        with sr.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source)
            audio = self.recognizer.listen(source)

        print("[INFO]: Riconoscimento in corso...")
        try:
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


                            response = self.responses[topic]
                            response = response[self.resp_cycle[topic][1]]
                            self.resp_cycle[topic][1] = (self.resp_cycle[topic][1]+1)%(self.resp_cycle[topic][0])
                            print("[INFO]: Trovata parola chiave: %s, Risposta: %s", keyword, response)

                            self.say_something(response, response)
                        
                        else:
                            try:
                                res = rospy.wait_for_message('/POI/move/status', String, timeout=None)
                                print(res)
                                if res.data == "not_found":
                                    response = "no_POI"
                            except rospy.ROSException:
                                res = "error"
                                rospy.logerr("Timeout reached")
                                self.say_something("Errore, non ho stato trovato niente, prova a ripetere")
                                
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
        speech_recognizer = SpeechRecognizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

