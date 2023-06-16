#!/usr/bin/env python

'''
HKU-Astar
Based on conventional detection algorithm
'''

# Camera parameters: /opt/MVS/Samples/64/Python/ParametrizeCamera_LoadAndSave

import time
import argparse
import cv2 as cv
import numpy as np
import rospy as ros
from tools import *
import message_filters
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from msgs.msg import Armors

### Hyperparameters ###
IMG_W = 1440
IMG_H = 1080
PITCH_OFFSET = 0.025 # for raising the head
MAX_YAW = 0.070
MAX_PITCH = 0.03
MIN_YAW = 0.005
MIN_PITCH = 0.005
YAW_RANGE_1 = 400 # velocity mode
YAW_RANGE_2 = 150 # acceleration mode
PITCH_RANGE = 200
MAX_YAW_ACC = 0.110
MIN_YAW_ACC = 0.040
MAX_PITCH_ACC = 0.035
MIN_PITCH_ACC = 0.010
TARGET_ZONE = 8 # radius in pixels
MIN_TARGET_AREA = 666
TARGET_AREA_EPS = 100
MODE_EPS = 3
### Hyperparameters ###

### Shooting ###
parser = argparse.ArgumentParser(description='Auto-aiming Mode')
parser.add_argument('--shoot', action='store_true', help='shoot')
args = parser.parse_args()
FORCE_SHOOT = True
SHOOT_X = IMG_W // 2
SHOOT_Y = IMG_H // 2 
### Shooting ###
 
time0 = time.time()

class AutoAiming:
    def __init__(self):
        ros.init_node('auto_aiming')
        self.cv_bridge = CvBridge() # tool to convert cv2 images to ros image

        self.controller = Controller()

        self.img_pub = ros.Publisher('/annotated_image', Image, queue_size=10) # publish labeled image
        self.coords_pub = ros.Publisher('/target_coordinates', Vector3, queue_size=10)
        self.tar_sub = message_filters.Subscriber('/armors_publisher', Armors) # get coordinate of armors in pixels
        # self.img_sub = message_filters.Subscriber('hikrobot_camera/rgb', Image) # get camera image
        # self.all_sub = message_filters.ApproximateTimeSynchronizer([self.img_sub, self.tar_sub], 1, 0.2)

        self.tar_sub.registerCallback(self.callback)

        self.last_act = [0, 0]
        self.last_tar = None # for target tracing, (x, y)
        self.last_tar_v = [0, 0]

        self.x_on_target = False

        self.foo = []

        self.mode = 'acceleration'

        ros.spin() # start

    def callback(self, armors):
        global time0
        self.foo.append(time.time() - time0)
        ros.loginfo('Time: ' + str(np.mean(self.foo[-60:])))

        img = armors.img
        img = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, -1)
        armors = armors.items

        tar, tar_bbox = self.pick_target(armors)
        ros.loginfo('Target: ' + str(tar) + ', Bounding Box:' + str(tar_bbox))
        
        target_xyz = Vector3()
        target_xyz.x = tar[0]
        target_xyz.y = tar[1]
        target_xyz.z = 0
        self.coords_pub.publish(target_xyz)

        if FORCE_SHOOT and tar != None :
            self.shoot_target(tar, tar_bbox)

        act, pred_tar, pred_tar_v = self.trace_target(tar, tar_bbox)
        # self.publish_annotated_image(img, armors, tar, pred_tar, act)

        self.last_tar = tar
        self.last_act = act
        self.last_tar_v = pred_tar_v
        time0 = time.time()

    def shoot_target(self, tar, tar_bbox):
        if SHOOT_X > tar_bbox[0] and SHOOT_X < tar_bbox[2] and SHOOT_Y > tar_bbox[1] and SHOOT_Y < tar_bbox[3]:
            self.controller.shoot(mode=1)

    def trace_target(self, tar, tar_bbox):
        if tar == None:
            return [0, 0], None, [0, 0]

        vx, vy = 0, 0 # predicted speed
        dx, dy = 0, 0 # action to be taken
        ax, ay = 0, 0 # acceleration
        tar_ax, tar_ay = 0, 0
        cx, cy = SHOOT_X, SHOOT_Y # target of aiming
        pred_tar = tar[:] # predicted coordinate in image

        ros.loginfo(f'Center: {cx}, Target: {tar_bbox[0]}-{tar_bbox[2]}')
        
        if self.x_on_target == True and (cx - tar_bbox[2]) * self.last_act[0] < 0 and (cx - tar_bbox[0]) * self.last_act[0] < 0:
            self.mode = 'velocity'
            # self.mode = 'acceleration' # for testing
        if self.x_on_target == True and (cx - tar_bbox[2]) * self.last_act[0] > 0 and (cx - tar_bbox[0]) * self.last_act[0] > 0:
            self.mode = 'acceleration'
        if cx >= tar_bbox[0] and cx <= tar_bbox[2]:
            self.x_on_target = True
        else:
            self.x_on_target = False

        ros.loginfo('Mode: ' + self.mode)

        if self.mode == 'acceleration':
            if self.last_tar != None: # speed estimation
                vx = tar[0] - self.last_tar[0]
                vy = tar[1] - self.last_tar[1]

            pred_tar[0] += int(vx * 30) # default:35
            pred_tar[1] += int(vy * 30)

            if abs(pred_tar[0] - cx) > TARGET_ZONE:
                ax = min((abs((pred_tar[0] - cx)) / YAW_RANGE_2) ** (6), 1) * MAX_YAW_ACC
                ax = max(ax, MIN_YAW_ACC)
                if pred_tar[0] > cx: 
                    ax = -ax

            if abs(pred_tar[1] - cy) > TARGET_ZONE:
                ay = min(abs((pred_tar[1] - cy) / PITCH_RANGE) ** (6), 1) * MAX_PITCH_ACC
                ay = max(ay, MIN_PITCH_ACC)
                if pred_tar[1] > cy: 
                    ay = -ay

            dx = min(self.last_act[0] + ax * 0.025, MAX_YAW)  # - tar_ax * 0.001, MAX_YAW)
            dy = min(self.last_act[1] + ay * 0.025, MAX_PITCH)

        if self.mode == 'velocity':
            if abs(tar[0] - cx) > TARGET_ZONE:
                dx = min((abs((tar[0] - cx)) / YAW_RANGE_1), 1) * MAX_YAW
                dx = max(dx, MIN_YAW)
                if tar[0] > cx: 
                    dx = -dx

            if abs(tar[1] - cy) > TARGET_ZONE:
                dy = min(abs((tar[1] - cy) / PITCH_RANGE), 1) * MAX_PITCH
                dy = max(dy, MIN_PITCH)
                if tar[1] > cy: 
                    dy = -dy

        self.controller.move(dx, -dy, PITCH_OFFSET, log=True)

        ros.loginfo('Last action: ' + str(self.last_act))

        return [dx, dy], [pred_tar[0], pred_tar[1]], [dx, dy]

    def pick_target(self, armors):
        '''
        current strategy: distance to last target, area of bounding box.
        should consider location of target!!!!
        '''
        if len(armors) == 0:
            return None, None
        area = []
        dist = []
        for armor in armors:
            area.append(armor.area)
            if self.last_tar != None:
                x0, y0 = self.last_tar
                dist.append(np.hypot(armor.cx - x0, armor.cy - y0))
        if self.last_tar != None:
            target = np.argmin(dist) # trace the last target by default
            if area[target] < MIN_TARGET_AREA and area[np.argmax(area)] - area[target] > TARGET_AREA_EPS: 
                target = np.argmax(area) # if the last target is too small and there is a larger one, then trace it
        else:
            target = np.argmax(area)
        tar = armors[target]
        x1, y1, x2, y2 = tar.x1, tar.y1, tar.x2, tar.y2
        ret = [tar.cx, tar.cy]
        return ret, [x1, y1, x2, y2]

    def publish_annotated_image(self, img, armors, tar, pred_tar, act):
        img = cv.line(img, (SHOOT_X, 0), (SHOOT_X, IMG_H - 1), color=(128, 128, 128), thickness=2)
        img = cv.line(img, (0, SHOOT_Y), (IMG_W - 1, SHOOT_Y), color=(128, 128, 128), thickness=2)
        for armor in armors:
            x1, y1, x2, y2 = armor.x1, armor.y1, armor.x2, armor.y2
            x1 = int(x1)
            x2 = int(x2)
            y1 = int(y1)
            y2 = int(y2)
            img = cv.rectangle(img, (x1, y1), (x2, y2), color=(0, 204, 102), thickness=2)
        if tar != None:
            img = cv.rectangle(img, (tar[0] - 6, tar[1] - 6), (tar[0] + 6, tar[1] + 6), color=(255, 17, 127), thickness=-1)
        if pred_tar != None:
            img = cv.line(img, (max(pred_tar[0] - 10, 0), max(pred_tar[1] - 10, 0)), (pred_tar[0] + 10, pred_tar[1] + 10), color=(122, 40, 203), thickness=2)
            img = cv.line(img, (pred_tar[0] + 10, pred_tar[1] - 10), (max(pred_tar[0] - 10, 0), max(pred_tar[1] + 10, 0)), color=(122, 40, 203), thickness=2)
        if act != None:
            img = cv.rectangle(img, (1, 1), (int(100 * act[0] / MAX_YAW), 30), color=(255, 27, 28), thickness=-1)

        img = cv.cvtColor(img, cv.COLOR_RGB2BGR)
        msg = self.cv_bridge.cv2_to_imgmsg(img, encoding='passthrough')
        self.img_pub.publish(msg)
            

if __name__ == '__main__':
    AutoAiming()

