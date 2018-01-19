#!/usr/bin/env python
# coding: utf-8
# Copyright (C) 2017 Toyota Motor Corporation
u"""認識結果を画像として表示するノード実装"""

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
from tmc_yolo2_ros.msg import Detections

COLORS_ = [(255, 0, 0), (0, 255, 0), (0, 0, 255),
           (255, 255, 0), (255, 0, 255), (0, 255, 255),
           (127, 0, 0), (0, 127, 0), (0, 0, 127),
           (127, 127, 0), (127, 0, 127), (0, 127, 127),
           (127, 127, 127)]


class Yolo2ImageNode(object):
    u"""認識結果を画像として表示するノード"""

    def __init__(self):
        self._bridge = CvBridge()
        self._input_image = None

        self._image_sub = rospy.Subscriber(
            '~image', Image, self._color_image_cb)
        self._result_sub = rospy.Subscriber(
            '~detections', Detections, self._result_cb)
        self._result_image_pub = rospy.Publisher(
            "~image_result", Image,
            queue_size=10)

    def _color_image_cb(self, data):
        # Don't do any work if nobody is listening
        if self._result_image_pub.get_num_connections() == 0:
            return
        try:
            self.width = data.width
            self.height = data.height
            self.encoding = data.encoding
            self._input_image = self._bridge.imgmsg_to_cv2(data,
                                                           self.encoding)
        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)

    def _result_cb(self, data):
        if self._input_image is None:
            return
        # Don't do any work if nobody is listening
        if self._result_image_pub.get_num_connections() == 0:
            return
        output_image = self._input_image
        detect_classes = []
        for detection in data.detections:
            if detection.class_name not in detect_classes:
                detect_classes.append(detection.class_name)
            cv2.rectangle(output_image,
                          (int(detection.x - detection.width / 2.0),
                           int(detection.y -
                               detection.height / 2.0)),
                          (int(detection.x + detection.width / 2.0),
                           int(detection.y +
                               detection.height / 2.0)),
                          COLORS_[detect_classes.index(
                              detection.class_name) % len(COLORS_)], 2)
            cv2.rectangle(output_image,
                          (int(detection.x),
                           int(detection.y - 20)),
                          (int(detection.x + 15 *
                               len(detection.class_name) + 5),
                           int(detection.y)),
                          (255, 255, 255), -1)
            cv2.putText(output_image, detection.class_name,
                        (int(detection.x), int(detection.y - 5)
                         ), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                        COLORS_[detect_classes.index(
                            detection.class_name) % len(COLORS_)], 2)

        self._result_image_pub.publish(
            self._bridge.cv2_to_imgmsg(
                output_image, encoding=self.encoding))
