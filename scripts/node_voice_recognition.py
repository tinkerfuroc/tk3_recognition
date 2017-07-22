#!/usr/bin/env python
# -*- coding:UTF-8 -*-
# File Name :
# Purpose :
# Creation Date : 21-07-2017
# Last Modified : Sat 22 Jul 2017 02:40:19 PM
# Created By : Jeasine Ma [jeasinema[at]gmail[dot]com]

from __future__ import print_function
from __future__ import division
from __future__ import unicode_literals

from struct import pack, unpack
import pyaudio
import speech_recognition as sr
import rospy
from std_msgs.msg import String, Bool

topic_voice_recog = 'voice_recog'
bing_speech_key = 'a9b9e452624d4769bfffd3eb9cb53c77'
noise_duration = 1
node_duration = 0.1

def main(*args, **kwargs):
    rospy.init_node("tk3_voice_recog")
    
    _s = rospy.Publisher(topic_voice_recog, String)
    rospy.loginfo(topic_voice_recog + "start!")
    rate = rospy.Rate(1/node_duration)

    r = sr.Recognizer()
    while not rospy.is_shutdown():
        with sr.Microphone() as source:
            r.adjust_for_ambient_noise(source, duration=noise_duration)
            audio = r.listen(source)
            rospy.loginfo("Finish a sentence!")

        res = r.recognize_bing(audio, key=bing_speech_key)
        rospy.loginfo("Bing thinks you said: {}".format(res))
        _s.publish(res)
        rate.sleep()


if __name__ == "__main__":
    main()	
