#!/usr/bin/env python
# -*- coding:UTF-8 -*-
# File Name : node_person_recognition.py
# Purpose :
# Creation Date : 21-07-2017
# Last Modified : Sat 22 Jul 2017 05:00:11 PM
# Created By : Jeasine Ma [jeasinema[at]gmail[dot]com]

from __future__ import print_function
from __future__ import division
from __future__ import unicode_literals


import base64
import json
import sys
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import requests
from tk3_recognition.srv import *
from std_msgs.msg import Int32
from geometry_msgs.msg import Polygon, Point32

srv_person_recog = "srv_person_recog"


def detect_face(img):
    subscription_key = '6f19ccc9edef445ca4a8f4d6eea624df' 
    uri_base = 'https://westcentralus.api.cognitive.microsoft.com/face/v1.0'

    # Request headers.
    headers = {
        'Content-Type': 'application/octet-stream',
        'Ocp-Apim-Subscription-Key': subscription_key,
    }

    # Request parameters.
    params = {
        'returnFaceId': 'true',
        'returnFaceLandmarks': 'false',
        #'returnFaceAttributes': 'age,gender,headPose,smile,facialHair,glasses,emotion,hair,makeup,occlusion,accessories,blur,exposure,noise',
        'returnFaceAttributes': 'age,gender,headPose',
    }

    # raw image to analyze.
    body = img.data

    try:
        # Execute the REST API call and get the response.
        response = requests.post(uri_base + "/detect?", headers=headers, data=body, params=params)
        data = response.text

        # 'data' contains the JSON data. The following formats the JSON data for display.
        parsed = json.loads(data)

    except Exception as e:
        print("[Errno {0}] {1}".format(e.errno, e.strerror))

    # handle parsed
    face_info = {
            "gender" : 0, # 0 for male
            "age" : 0,
            "pos" : [  # pos for diagonal
                (0, 0),
                (0, 0)
                ]
    }
    try:
        for face in parsed:
            face_info['gender'] = 0 if face['faceAttributes']['gender'] == 'male' else 1
            face_info['age'] = face['faceAttributes']['age']
            face_info['pos'] = [
                    (face['faceRectangle']['left'], face['faceRectangle']['top']),
                    (face['faceRectangle']['left'] + face['faceRectangle']['width'], 
                        face['faceRectangle']['top'] + face['faceRectangle']['height'])
                    ]
            yield face_info
    except Exception as e:
        raise StopIteration
    raise StopIteration

def handle_face_recog(req):
    ret = FaceRecogResponse()
    ret.face_amount = 0
    img = req.face_image 

    for info in detect_face(img):
        ret.face_gender.append(info['gender'])
        ret.face_age.append(info['age'])
        pos = Polygon()
        pos.points.append(Point32(info['pos'][0][0], info['pos'][0][1], 0))
        pos.points.append(Point32(info['pos'][1][0], info['pos'][1][1], 0))
        ret.face_pos.append(pos)
        ret.face_amount = ret.face_amount + 1
    
    rospy.loginfo("Detect {} faces!".format(ret.face_amount))
    return ret


def main(*args, **kwargs):
    global srv_person_recog
    
    rospy.init_node("tk3_person_recog")

    _s = rospy.Service(srv_person_recog, FaceRecog, handle_face_recog)
    rospy.loginfo(srv_person_recog + " start!")
    rospy.spin()
    

if __name__ == "__main__":
    main()

