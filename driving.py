#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os

class LaneDetection:
    def __init__(self):
        self.image = np.empty(shape=[0])
        self.bridge = CvBridge()
        self.motor = None

        self.CAM_FPS = 30
        self.WIDTH, self.HEIGHT = 640, 480

        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        time.sleep(3)
        os.system('killall -9 python rosout')
        sys.exit(0)

    def preprocess_image(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        return edges

    def detect_lines(self, img):
        lines = cv2.HoughLinesP(img, 1, np.pi / 180, threshold=50, minLineLength=30, maxLineGap=100)
        if lines is None:
            return None
        return lines

    def get_steering_angle(self, lines, img_shape):
        if lines is None or len(lines) == 0:
            return 0

        max_length = 0
        max_slope = 0
        center_x = img_shape[1] // 2

        for line in lines:
            x1, y1, x2, y2 = line[0]
            length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            slope = (y2 - y1) / (x2 - x1 + 1e-6)

            if length > max_length and abs(slope) > 0.2:
                max_length = length
                max_slope = slope

        angle = np.arctan(max_slope) * 180 / np.pi
        deviation = center_x - (x1 + x2) // 2
        angle += np.arctan(deviation / img_shape[1]) * 180 / np.pi

        if max_slope > 0:
            angle *= -1

        return int(angle)

    def visualize_lines(self, img, lines):
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

    def start(self):
        rospy.init_node('auto_drive')
        self.motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        self.motor = xycar_motor()

        while not rospy.is_shutdown():
            if self.image.size == (self.WIDTH * self.HEIGHT * 3):
                lane_img = self.image.copy()

                edges = self.preprocess_image(lane_img)

                lines = self.detect_lines(edges)

                self.visualize_lines(lane_img, lines)

                angle = self.get_steering_angle(lines, lane_img.shape)

                self.motor.angle = angle
                self.motor.speed = 20

                self.motor_pub.publish(self.motor)

                cv2.imshow("차선 인식", lane_img)

            cv2.waitKey(1)

    def img_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")


if __name__ == '__main__':
    lane_detection = LaneDetection()
    lane_detection.start()
