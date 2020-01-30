#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from utils.param import Param
from algorithms import lane, sign, obstacle
from control import car_control


class Node:
    def __init__(self, param):
        self.bridge = CvBridge()
        self.param = param
        rospy.init_node(param.node_name, anonymous=True)
        self.obstacle_detector = obstacle.detector.ObstacleDetector()
        self.sign_detector = sign.detection.SignDetector(self.param.model_sign_path)
        self.lane_segmenter = lane.segmentation.AutoEncoder(self.param.model_lane_path)
        self.control = car_control.CarController(self.param)

        self.control_count = 0
        self.sign_detect_count = 0
        self.obstacle_detect_count = 0

        self.sign = (0, ())
        self.danger_zone = (0, )
        rospy.Subscriber("/team812/camera/rgb/compressed", CompressedImage,
                         callback=self.callback_control, queue_size=1)
        rospy.Subscriber("/team812/camera/rgb/compressed", CompressedImage,
                         callback=self.callback_detect_sign, queue_size=1)
        rospy.Subscriber("/team812/camera/depth/compressed", CompressedImage,
                         callback=self.callback_detect_obstacle, queue_size=1)

    def callback_control(self, data):
        self.control_count += 1
        if self.control_count % 2 == 0:
            self.control_count = 0
            img = np.fromstring(data.data, np.uint8)
            image_np = cv2.imdecode(img, cv2.IMREAD_COLOR)
            image_np = cv2.cvtColor(image_np, cv2.COLOR_HSV2BGR)
            self.control.control(image_np, self.sign, self.danger_zone)

    def callback_detect_sign(self, data):
        self.sign_detect_count += 1
        if self.sign_detect_count % 3 == 0:
            self.sign_detect_count = 0
            img = np.fromstring(data.data, np.uint8)
            image_np = cv2.imdecode(img, cv2.IMREAD_COLOR)
            self.sign = self.sign_detector.detect_sign(image_np)

    def callback_detect_obstacle(self, data):
            self.obstacle_detect_count += 1
            if self.obstacle_detect_count % 2 == 0:
                self.obstacle_detect_count = 0
                img = np.fromstring(data.data, np.uint8)
                image_np = cv2.imdecode(img, cv2.IMREAD_COLOR)
                self.danger_zone = self.obstacle_detector.combine(image_np)


if __name__ == '__main__':
    param = Param()
    node = Node(param)
    rospy.spin()