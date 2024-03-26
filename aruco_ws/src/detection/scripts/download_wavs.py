import os

from collections import deque
import json

respjson = "../json/responses.json" # Name of the json file

from gtts import gTTS
import subprocess 

def prerecord_responses():
    with open(respjson) as json_file:
        resp2text = json.load(json_file)
        for r in resp2text:
            tts = gTTS(text=resp2text[r], lang='it')    

            mp3name =  "../wavs/ARI-wavs" + r +  ".mp3"
            wavname =  "../wavs/ARI-wavs" + r +  ".wav"
            tts.save(mp3name)

            subprocess.call(['ffmpeg', '-i', mp3name, wavname])

prerecord_responses()