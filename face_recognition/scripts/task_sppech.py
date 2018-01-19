#!/usr/bin/env python

"""
Task 3 as specified in 5.3.3 of the Rulebook
"""

import rospy
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty
from tmc_msgs.msg import Voice
from hark_msgs.msg import HarkSource
import time
import math
import functools
import random
from hsrb_interface import Robot
from villa_task import tts
# from villa_sound_localization.motionhandler import MotionHandler
# from villa_task.task3.personangle import LocatePerson
from villa_helpers.fast_move import FastMove
import re
from hsrb_interface import geometry
# from spr_qa.xml_kbase import XMLKnowledgeBase
# from spr_qa.spr_qa_main import QuestionAnswerer
from villa_task.task3.personangle import LocatePerson

speech_time = 0
speech_transcript = ''
listen = None

speech = None
unanswered = False
speak_time = 0

 
# def talker(word):
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rate = rospy.Rate(10) # 10hz
#     pub.publish(word)

def update_face(s):
    global speech
    global stop_speech
    global stop_mov
    global remember
    sentence = " q "
    # if s.data == "minkyu" or s.data == "nicolass":
    stop_speech = True
    sentence = "Hello  " + s.data
    if sentence != remember or stop_mov == False:
        stop_mov = True
        speech.say(sentence)
        remember = sentence
    stop_speech = False


def update_speech(s):
    print("speech running")
    global speech_time
    global speech_transcript
    global question_count
    global listen
    global speech
    global unanswered


    speech_time = rospy.get_time()
    speech_transcript = s.data
    f = open('/home/nicolas/Desktop/NewFile2.txt', 'a')
    f.write(speech_transcript + '\n')
       
    words = []
    ind_temp = 0

    for word in speech_transcript.split():
        print word
        if (word == "follow" or word == "stop") and stop_speech == False :
            if word == "follow":
                stop_mov = True
            if word == "stop":
                stop_mov = False
            print word," ORDER IS UNDERSTOOD"
            try:
                talker(word)
            except rospy.ROSInterruptException:
                pass      

        
def move_robot(omni_base, azimuth):

    print("Begin moving robot")
    omni_base.go(0., 0., math.pi * azimuth / 180., 100., relative=True)


def __localization_callback(omni_base, detect_person, data):
    global speech_time
    global stop_mov
    global stop_speech
    if len(data.src) <= 0:
        return
    # Get max power source
    src = max(data.src, key=lambda x: x.power)
    azimuth = src.azimuth
    print("Detected source with power: " + str(src.power))
    print("Time difference:", rospy.get_time() - speech_time)
    wait_until = rospy.get_time() + 5
    # while rospy.get_time() < wait_until:
    #     if abs(rospy.get_time() - speech_time) < 5:
    #         break
    #     rospy.sleep(0.1)
    # if abs(rospy.get_time() - speech_time) < 5 :
    if stop_speech == False:
        print("Allowing to move")
        move_robot(omni_base,azimuth)
        detect_person.turn_to_person()
    else :
        print(stop_speech)
    # else :
    #     print("You shall not move")


def act(robot):
    global listen
    global speech
    global unanswered
    global speech_time
    global question_count
    global turning
    global speak_time
    global stop_speech
    global remember
    remember = " "

    Turned = False
    stop_mov = False
    stop_speech = False
    whole_body = robot.get('whole_body')
    omni_base = FastMove(robot.get('omni_base'))
    detect_person = LocatePerson(omni_base)

    speech = tts.TextToSpeech()
    
    whole_body.move_to_neutral()
    stop_head = rospy.ServiceProxy("/viewpoint_controller/stop", Empty)
    stop_head()
    speech_time = rospy.get_time()
    rospy.Subscriber("/HarkSource",HarkSource,functools.partial(__localization_callback,omni_base,detect_person),queue_size=1)
    

    speech_time = rospy.get_time()
    # rospy.sleep(0.5) # natural sounding pause
    # speech.say("Trying to run speech")
    # speech_time = rospy.get_time()
    # rospy.sleep(0.5)
    print('Subscribing to speech detection..')
    speech_detection = rospy.Subscriber('/villa/speech_transcripts', String, update_speech)
    face_detection = rospy.Subscriber('/face_detected_name', String, update_face)

    # Start listening to localization
    # print("Starting motion handler")
    # motionhandler.start()

    rospy.spin()


print("Process starts")

with Robot() as robot:
    act(robot)

