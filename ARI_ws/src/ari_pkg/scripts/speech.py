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
            
            for r in self.responses:
                self.resp_cycle[r] = [len(self.responses[r]), 0]

        # Inizializza il recognizer di speech_recognition
        self.recognizer = sr.Recognizer()

        # Avvia l'ascolto del microfono
        self.listen_microphone()



    def listen_microphone(self):
        rospy.loginfo("In ascolto... Parla pure!")
        with sr.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source)
            audio = self.recognizer.listen(source)

        rospy.loginfo("Riconoscimento in corso...")
        try:
            spoken_text = self.recognizer.recognize_google(audio, language='it-IT').lower()
            print("Hai detto: " + spoken_text)
            for topic, keywords in self.topics.items():
                for keyword in keywords:
                    if keyword.lower() in spoken_text:
                        # IN CASO IL TOPIC SIA "ARRIVEDERCI" ESCI DAL PROGRAMMA
                        if topic is "addio":
                            continue_listen = False
                            response = "addio"
                            os.system("aplay " + os.path.join(self.current_dir,self.path_wavs) + response + ".wav")
                            exit()
                        # Altrimenti puo' essere un topic di tipo solo speech oppure anche di movimento
                        else:
                                
                            response = self.responses.get(topic, "No")
                            print(response)
                            if response is not "No":
                                response = response[self.resp_cycle[topic][1]]
                                self.resp_cycle[topic][1] = (self.resp_cycle[topic][1]+1)%(self.resp_cycle[topic][0])
                                if topic == "goto":
                                    self.check_goto.publish(String(spoken_text))
                                    # res accepted:
                                    # "ok" : all done successfully
                                    # "not_found" : no POI found 
                                    # "error" : something went wrong --> TODO make a wav for this 
                                    # TODO: make a directory only for the responses of the movement (wavs)
                                    try:
                                        res = rospy.wait_for_message('/POI/move/status', String, timeout=None)
                                        print(res)
                                        if res.data == "not_found":
                                            response = "no_POI"
                                    except rospy.ROSException:
                                        # TODO play wav for the unknown error
                                        res = "error"
                                        rospy.logerr("Timeout reached")
                                        self.say_something("Errore, non ho stato trovato niente, prova a ripetere")

                            if response is not "error":
                                rospy.loginfo("Trovata parola chiave: %s, Risposta: %s", keyword, response)
                        

                        os.system("aplay " + os.path.join(self.current_dir,self.path_wavs) + response + ".wav")
                        print("continuo...")

                        # Riproduci la risposta dall'altoparlante
                        #os.system('echo ' + response)
                            

            rospy.loginfo("Nessuna corrispondenza trovata.")
        except sr.UnknownValueError:
            rospy.loginfo("testo non riconosciuto")
        except sr.RequestError as e:
            rospy.loginfo("errore durante la richiesta a Google")
        
        self.listen_microphone()
        
    def say_something(self, text):
        tts = gTTS(text=text, lang='it')
        tts.save("response.mp3")
        
        subprocess.call(['ffmpeg', '-i', "response.mp3", "response.wav"])
        
        os.system("aplay response.wav")
        os.system("rm response.wav")
        os.system("rm response.mp3")

if __name__ == '__main__':
    try:
        speech_recognizer = SpeechRecognizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

