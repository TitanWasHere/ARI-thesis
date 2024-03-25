#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from actionlib import SimpleActionClient
from std_msgs.msg import Header

import os
import speech_recognition as sr
#from gtts import gTTS

from collections import deque
import json

class ARI:
    
    def __init__(self):
        self.respjson = "../json/responses.json" # Name of the json file
        self.topicjson = "../topics.json" # Name of the json file where are the topic on listening

        self.startkwd = "chiara" # Start keyword
        self.forsestartkwd = "ciao chiara"
        self.startkwdalt = "cara"

        self.wavs = "" # Name of the wav file
        
        ## global maps (multi-dialogue)
        self.seen = {} # Seen questions
        self.resp2text = {} # Response id to text
        self.topickwd = {} # Topic keyword

        ## Update for each dialogue
        self.used = {} # Answer used
        self.responses = {}

        self.turnno = -1 # Turn in the current dialogue
        self.diano = 0 # Dialogue number
        self.noidea = 0

        self.onhold = True

        # Opening JSON file, read topics and responses
        with open(self.respjson) as json_file:
            self.resp2text = json.load(json_file)
        with open(self.topicjson) as json_file:
            self.topickwd = json.load(json_file)

        self.recognizer_instance = sr.Recognizer()

        while not rospy.is_shutdown():
            current_question = self.get_current_question().lower()
            if self.StopListening(current_question):
                break
            if current_question == "":
                continue
            if current_question == " ":
                continue
            if self.forsestartkwd in current_question:
                self.onhold = False
                self.turnno = -1
                self.StartDialogue()
                continue
            if self.startkwd in current_question:
                self.onhold = False
            if self.startkwdalt in current_question:
                self.onhold = False

            if self.onhold:
                continue

            if self.turnno == -1:
                self.StartDialogue()
                continue

            text = " " + current_question + " "
            (topic, answer) = self.produceAnswer(text)
            self.giveAnswer(answer)

            if topic == "DEFAULT":
                self.noidea += 1

            self.turnno += 1
            if self.FinishDialogueNow(text):
                self.onhold = True
                self.turnno = -1
                if self.noidea > 2:
                    self.giveAnswer("devoimparare")
                self.giveAnswer("grazie")


    def get_current_question(self):
        text = ''
        with sr.Microphone() as source:
            self.recognizer_instance.adjust_for_ambient_noise(source)
            audio = self.recognizer_instance.listen(source)
        try:
            text = self.recognizer_instance.recognize_google(audio, language='it-IT')
        except Exception as e:
            print(e)

        print("heard: " + text)
        return text
    
    def FinishDialogueNow(self, text):
        if self.noidea > 2:
            return True
        if self.turnno < 2:
            return False
        if 'ciao' in text.lower():
            return True
        if 'arrivederci' in text.lower():
            return True
        return False
    
    def StopListening(self, text):
        if 'basta' in text.lower():
            self.onhold = True
            self.giveAnswer("ciao")
            self.giveAnswer("grazie")
            self.giveAnswer("devoimparare")
            self.giveAnswer("cisentiamo")
            return True
        return False
    
    def InputTopics(self):
        self.responses = {}
        for t in self.topickwd:
            topic = t['topic']
            self.responses['topic'] = deque(t['responses'])
        self.responses['DEFAULT'] = deque(['nessunidea'])

    def ExtractTopic(self, text):
        tlo = text.lower()

        for t in self.topickwd:
            for k in t['keywords']:
                if k in tlo:
                    return t['topic']
                
        return 'DEFAULT'
    
    def QLookUp(self, text):
        if text.lower() in self.seen:
            return self.seen[text.lower()]
        return ''
    
    def ALookUp(self, text):
        if text == "AUX":
            return True
        if text.lower() in self.used:
            return True
        return False
    
    def produceAnswer(self, text):
        if self.QLookUp(text):
            return self.QLookUp(text)
        
        topic = self.ExtractTopic(text)
        if self.turnno < 2: 
            if topic == "DEFAULT" or topic == "ciao":
                topic = "ari"
        
        answer = ''
        if topic not in self.responses or len(self.responses[topic]) == 0:
            topic = 'DEFAULT'

        if topic == 'DEFAULT':
            answer = self.responses[topic][0]
        else:
            answer = 'AUX'
            while len(self.responses[topic]) > 0 and self.ALookUp(answer) == True:
                answer = self.responses[topic].popleft()
                
            if self.ALookUp(answer) == True:
                topic = 'DEFAULT'
                answer = self.responses[topic][0]
            else:
                self.used[answer.lower()] = True

        self.seen[text.lower()] = (topic, answer)

        return (topic, answer)
    
    def StartDialogue(self):
        self.InputTopics()
        self.giveAnswer("startmsg")
        self.used = {}
        if self.diano > 0:
            self.giveAnswer("paanc")
        self.diano += 1
        self.turnno = 0
        self.noidea = 0

    def giveAnswer(self, text):
        if text in self.resp2text:
            os.system("aplay " + self.wavs + text + ".wav")
            return
        rospy.loginfo("cannot pronounce:" + text)

def main():
    rospy.init_node('listener', anonymous=True)
    sub = ARI()

if __name__ == '__main__':
    main()
    
        

            