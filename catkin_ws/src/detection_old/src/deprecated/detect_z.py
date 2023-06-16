#!/usr/bin/env python

'''
HKU-Astar
'''

import torch
import cv2 as cv
import numpy as np
import rospy as ros
from tools import *
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3

### Hyperparameters ###
IMG_W = 1920
IMG_H = 1080
OFFSET = 0.025
MAX_YAW = 0.080
MIN_YAW = 0.025
MAX_PITCH = 0.025
MIN_PITCH = 0.010
TARGET_ZONE = 8 # radius in pixels
MIN_TARGET_AREA = 666
TARGET_AREA_EPS = 100
YAW_RANGE = 150
PITCH_RANGE = 100
### Hyperparameters ###
 
class AutoAiming:
    def __init__(self):
        ros.init_node('auto_aiming')
        self.cv_bridge = CvBridge() # tool to convert cv2 images to ros image
        self.predictor = PredictSpeed()
        self.detector = ArmorDetector()
        self.controller = Controller()
        self.img_pub = ros.Publisher('/annotated_image', Image, queue_size=10) # publish labeled image
        self.tar_pub = ros.Publisher('/target_coordinates', Vector3, queue_size=10) # publish coordinate of target in pixels
        ros.Subscriber('camera/color/image_raw', Image, self.callback) # get camera image
        self.last_tar = None # for target tracing, (x, y, name)
        self.last_act = [0, 0]
        self.foo = [[], []]
        ros.spin() # start

    def callback(self, img):
        # ros.loginfo('Received image. Image width: ' + str(img.width) + 'Image height: ' + str(img.height))
        img = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, -1)
        res = self.detector.inference(img)
        tar = self.pick_target(res)
        ros.loginfo('Target: ' + str(tar))
        predicted_tar, act = self.trace_target(tar, res)
        self.publish_annotated_image(img, res, tar, predicted_tar, act)
        self.last_tar = tar
        self.last_act = act

    def trace_target(self, tar, labels=None):
        '''
        todo: consider enemy's speed
        '''
        if tar == None:
            msg = Vector3()
            msg.x, msg.y, msg.z = -1, -1, -1
            self.tar_pub.publish(msg)
            return None, [0, 0]

        vx, vy = 0, 0 # predicted speed
        dx, dy = 0, 0 # action to be taken
        ax, ay = 0, 0 # acceleration
        cx, cy = IMG_W / 2, IMG_H / 2
        predicted_tar = tar[:]

        if self.last_tar != None and tar[-1] == self.last_tar[-1]:
            predicted_tar[0], predicted_tar[1], vx, vy = self.predictor.predict(tar[:2]) # Kalman Filter
        else:
            self.predictor.reset(tar[:2])

        if self.last_tar != None:
            self.foo[0].append(vx)
            self.foo[1].append(vy)
            print('        ', np.mean(self.foo[0]), np.mean(self.foo[1]))

        predicted_tar[0] += int(vx * 30) - int(self.last_act[0] * 20)
        predicted_tar[1] += int(vy * 30) - int(self.last_act[1] * 20)

        if abs(predicted_tar[0] - cx) > TARGET_ZONE:
            # ax = min(np.sqrt(abs((predicted_tar[0] - cx)) / YAW_RANGE) ** 4, 1) * MAX_YAW
            ax = min((abs((predicted_tar[0] - cx)) / YAW_RANGE), 1) * MAX_YAW
            ax = max(ax, MIN_YAW)
            if predicted_tar[0] > cx: 
                ax = -ax
        if abs(predicted_tar[1] - cy) > TARGET_ZONE:
            ay = min(np.sqrt(abs((predicted_tar[1] - cy)) / PITCH_RANGE), 1) * MAX_PITCH
            ay = max(ay, MIN_PITCH)
            if predicted_tar[1] > cy: 
                ay = -ay

        dx = self.last_act[0] + ax * 0.01
        dy = self.last_act[1] - ay * 0.01

        self.controller.move(dx, dy, OFFSET, log=True)

        msg = Vector3()
        msg.x, msg.y, msg.z = tar[0], tar[1], dx
        self.tar_pub.publish(msg)

        # dx, dy = 0, 0

        ros.loginfo('Action: ' + str(dx) + ' ' + str(dy))
        ros.loginfo('Last action: ' + str(self.last_act))

        return [predicted_tar[0], predicted_tar[1]], [dx, dy]

    def pick_target(self, labels):
        '''
        current strategy: distance to last target, area of bounding box.
        should consider location of target!!!!
        '''
        filtered_labels = []
        for label in labels:
            if label[-3] > 0.5:
                print(label)
                filtered_labels.append(label)
        labels = filtered_labels
        if len(labels) == 0:
            return None
        area = []
        dist = []
        for label in labels:
            x1, y1, x2, y2 = label[:4]
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            area.append((x2 - x1) * (y2 - y1))
            if self.last_tar != None:
                x0, y0 = self.last_tar[:2]
                dist.append(np.hypot(cx - x0, cy - y0))
        if self.last_tar != None:
            target = np.argmin(dist) # trace the last target by default
            if area[target] < MIN_TARGET_AREA and area[np.argmax(area)] - area[target] > TARGET_AREA_EPS: 
                target = np.argmax(area) # if the last target is too small and there is another armor with significantly larger area, then trace a new one
        else:
            target = np.argmax(area)
        x1, y1, x2, y2 = labels[target][:4]
        ret = [int((x1 + x2) / 2), int((y1 + y2) / 2), labels[target][-1]]
        return ret

    def publish_annotated_image(self, img, labels, tar, predicted_tar, act):
        img = cv.line(img, (IMG_W // 2, 0), (IMG_W // 2, IMG_H - 1), color=(128, 128, 128), thickness=2)
        img = cv.line(img, (0, IMG_H // 2), (IMG_W - 1, IMG_H // 2), color=(128, 128, 128), thickness=2)
        for label in labels:
            x1, y1, x2, y2 = label[:4]
            x1 = int(x1)
            x2 = int(x2)
            y1 = int(y1)
            y2 = int(y2)
            img = cv.rectangle(img, (x1, y1), (x2, y2), color=(0, 204, 102), thickness=2)
            img = cv.putText(img, label[-1], (x1, y1-10), fontFace=cv.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(0, 204, 102), thickness=2)
        if tar != None:
            img = cv.rectangle(img, (tar[0] - 5, tar[1] - 5), (tar[0] + 5, tar[1] + 5), color=(255, 17, 127), thickness=-1)
        if predicted_tar != None:
            img = cv.line(img, (predicted_tar[0] - 10, predicted_tar[1] - 10), (predicted_tar[0] + 5, predicted_tar[1] + 5), color=(122, 40, 203), thickness=2)
            img = cv.line(img, (predicted_tar[0] + 10, predicted_tar[1] - 10), (predicted_tar[0] - 5, predicted_tar[1] + 5), color=(122, 40, 203), thickness=2)
        if act != None:
            img = cv.rectangle(img, (1, 1), (int(100 * act[0] / MAX_YAW), 30), color=(255, 27, 28), thickness=-1)

        img = cv.cvtColor(img, cv.COLOR_RGB2BGR)
        msg = self.cv_bridge.cv2_to_imgmsg(img, encoding='passthrough')
        self.img_pub.publish(msg)
            

if __name__ == '__main__':
    AutoAiming()

