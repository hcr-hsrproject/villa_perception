#!/usr/bin/env python
import sys
import roslib
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from keyboard.msg import Key
from cv_bridge import CvBridge, CvBridgeError
from os import listdir
from os.path import isfile, join, exists, abspath, dirname
import numpy as np
import std_msgs.msg

import tensorflow as tf
from rude_carnie.model import select_model, get_checkpoint
from rude_carnie import paths
from rude_carnie.detect import face_detection_model
from face_recognition_ros.srv import *


class FaceDectectors_photo(object):
    def __init__(self, wait=0.0):
        image_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
        rospy.Subscriber(image_topic, Image, self.image_callback)
        s = rospy.Service('/take_head_photo_srv', take_photo, self.Isrunnging)
        self.bridge = CvBridge()
        self.num_photo = 0
        self.takepicture = False

    def Isrunnging(self, req):
        # print(self.detected_msg.data)
        self.takepicture = True
        cv2.imwrite("face-" + str(self.num_photo) + ".jpg", self.cv2_img)
        self.savedname = "face-" + str(self.num_photo) + ".jpg"
        print('save picture')
        self.num_photo = self.num_photo + 1
        return self.savedname

    def listener(self):
        rospy.spin()

    def crop_face(self, image_data):
        image_batch = image_data
        files = []
        face_detect = face_detection_model('/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml')
        face_files = face_detect.run(image_data)
        # print(face_files)
        return face_files

    def image_callback(self, msg):
        # print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            self.cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            facelists = self.crop_face(self.cv2_img)

        except CvBridgeError, e:
            print("Image callaback error")
            print(e)

        # rospy.spin()


if __name__ == '__main__':
    rospy.init_node('take_head_photo_service')
    print("Initialize node")
    Face_manager = FaceDectectors_photo()
    print("Take photo service created")
    Face_manager.listener()
