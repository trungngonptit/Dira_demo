#!/usr/bin/env python

import math
import time
import rospy
from std_msgs.msg import Float32, String, Bool
from scripts.algorithms import lane, sign, obstacle


class CarController:
    def __init__(self, param):
        rospy.init_node(param.node_name, anonymous=True)
        self.pub_speed = rospy.Publisher("/team812/set_speed", Float32, queue_size=1)
        self.pub_steer = rospy.Publisher("/team812/set_angle", Float32, queue_size=1)
        self.param = param
        self.lane_segmenter = lane.segmentation.AutoEncoder(self.param.model_lane_path)
        self.obstacle_detector = obstacle.detector.ObstacleDetector()
        self.sign_detector = sign.detection.SignDetector(self.param.model_sign_path)
        self.lane_segmenter = lane.segmentation.AutoEncoder(self.param.model_lane_path)
        self.h = 160
        self.w = 320
        self.sign_last_detected = 0
        self.sign_type = 0
        self.current_speed = 0
        self.time_start = time.time()

    def control(self, img, sign, danger_zone):
        now = time.time()
        speed = 0
        steer_angle = self.cal_steer_angle(img, sign, danger_zone)
        if now - self.time_start > self.param.delay_time and self.current_speed == 0:
                self.current_speed = self.param.max_speed
        else:
            steer_angle = 0
        if self.current_speed != 0:
            speed = max(self.param.min_speed,
                        self.current_speed -
                        (self.param.base_speed - self.param.min_speed) *
                        abs(steer_angle ** 2) /
                        (self.param.max_steer_angle ** 2))

            if self.current_speed < self.param.max_speed:
                self.current_speed += 0.5
        self.pub_speed.publish(speed)
        self.pub_steer.publish(steer_angle)

    def cal_steer_angle(self, img, sign, danger_zone):
        now = time.time()
        if sign[0] != 0:
            self.sign_last_detected = time.time()
            self.sign_type = sign[0]

        self.lane_segmenter.roi = 0.62
        points = self.lane_segmenter.get_points(img)
        middle_pos = (points[0] + points[1]) / 2
        if points[0] == points[1] == 150:
            print 'lost lane'
            self.lane_segmenter.roi = 0.4
            points = self.lane_segmenter.get_points(img)
            middle_pos = (points[0] + points[1]) / 2

        if danger_zone != (0, 0):
            center_danger_zone = int((danger_zone[0] + danger_zone[1]) / 2)
            if danger_zone[0] < middle_pos < danger_zone[1]:
                if middle_pos < center_danger_zone:
                    middle_pos = danger_zone[0]
                else:
                    middle_pos = danger_zone[1]

        if self.param.time_decay*self.param.turning_time \
                < now - self.sign_last_detected < \
                self.param.turning_time:
            self.lane_segmenter.roi = 0.95
            points = self.lane_segmenter.get_points(img)
            middle_pos = (points[0] + points[1]) / 2
            if self.sign_type != 0:
                if self.sign_type == 1:
                    middle_pos = self.w
                if self.sign_type == 2:
                    middle_pos = 0

        distance_x = middle_pos - self.w/2
        distance_y = self.h - points[2]
        steer_angle = math.atan(float(distance_x) / distance_y) * 180 / math.pi
        return steer_angle