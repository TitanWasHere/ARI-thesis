#/usr/bin/env python

import rospy
import json
import speech_recognition as sr
import os



class SpeechRecognizer:
    def __init__(self):
        self.continue_listen = True
        rospy.init_node('speech_recognizer')
        self.path_wavs = "../wavs/andre/"
        self.resp_cycle = {}
        # Carica il file JSON con i topic
        with open('topics.json', 'r') as file:
            self.topics = json.load(file)
            
        # Carica il file JSON con le risposte
        with open('responses.json', 'r') as file:
            self.responses = json.load(file)
            
            for r in self.responses:
                self.resp_cycle[r] = [len(self.responses[r]), 0]

        # Inizializza il recognizer di speech_recognition
        self.recognizer = sr.Recognizer()

        # Avvia l'ascolto del microfono
        self.listen_microphone()

    def listen_microphone(self):
        while self.continue_listen is True:
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
                            if topic is "arrivederci":
                                continue_listen = False
                            else:
                                
                                response = self.responses.get(topic, "No")
                                print(response)
                                if response is not "No":
                             
                                    # [ len , 0 e/o i ]
                                    response = response[self.resp_cycle[topic][1]]
                                    # Dato che response --> { "topic" : [ "resp_1" , "resp_2", ... , "resp_i" , ... , "resp_n-1" ]
                                    # Allora salvo in un var json:
                                    # { "topic" : [ n , i ] }
                                    self.resp_cycle[topic][1] = (self.resp_cycle[topic][1]+1)%(self.resp_cycle[topic][0])
                            
                                rospy.loginfo("Trovata parola chiave: %s, Risposta: %s", keyword, response)
                            

                            os.system("aplay " + self.path_wavs + response + ".wav")
                            print("continuo...")

                            # Riproduci la risposta dall'altoparlante

                            #os.system('echo ' + response)
                            

                rospy.loginfo("Nessuna corrispondenza trovata.")
            except sr.UnknownValueError:
                rospy.loginfo("testo non riconosciuto")
            except sr.RequestError as e:
                rospy.loginfo("errore durante la richiesta a Google")
        
            self.listen_microphone()
            

if __name__ == '__main__':
    try:
        speech_recognizer = SpeechRecognizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

