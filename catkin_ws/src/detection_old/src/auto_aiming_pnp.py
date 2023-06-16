#!/usr/bin/env python3
import numpy as np
import rospy as ros
import cv2 as cv
from geometry_msgs.msg import Vector3
from msgs.msg import Armors
from gimbal_controller import Controller
from tools import pixel_to_cam, PredictSpeed
import math
import time

with open('params/camera_old_8mm.npy', 'rb') as f:
    camera_mtx = np.load(f)
    camera_dist = np.load(f)

PITCH_OFFSET = 0.00
controller = Controller()
predictor = PredictSpeed()
center = np.zeros(3, dtype=np.float32)
    
def callback(armors):
    time0 = time.time()
    if armors.count == 0:
        return
    print('Get armors:', armors.count)
    armor = armors.items[0]
    p_img = np.array([[armor.x1, armor.y1], [armor.x2, armor.y2], [armor.x3, armor.y3], [armor.x4, armor.y4]], dtype=np.float32)
    print('Got armor position at:', p_img[0], p_img[1], p_img[2], p_img[3])
    rvec, tvec = pixel_to_cam(p_img, camera_mtx, camera_dist)
    print('Position in camera coordinate system:', tvec[0], tvec[1], tvec[2])
    predicted_tvec = predictor.predict(tvec)[:3]
    print('Predicted position:', predicted_tvec)
    predicted_tvec -= center
    if predicted_tvec[2] == 0:
    	return
    #yaw_angle = math.atan(tvec[0] / tvec[2])
    #pitch_angle = math.atan(tvec[1] / tvec[2])
    yaw_angle = math.atan(predicted_tvec[0] / predicted_tvec[2])
    pitch_angle = math.atan(predicted_tvec[1] / predicted_tvec[2])
    controller.move_gimbal(-yaw_angle, 0, PITCH_OFFSET)
    if yaw_angle > 0:
        print('Yaw:', yaw_angle / np.pi * 180 , 'degrees right', end='')
    else:
        print('Yaw:', -yaw_angle / np.pi * 180 , 'degrees left', end='')
    if pitch_angle > 0:
        print(', Pitch:', pitch_angle / np.pi * 180, 'degrees down')
    else:
        print(', Pitch:', -pitch_angle / np.pi * 180, 'degrees up')
    time1 = time.time()
    print(time1 - time0)

def adjustment_callback(adjustment):
    global center
    center += adjustment.x, adjustment.y, adjustment.z
    print('Received adjustment:', adjustment.x, adjustment.y, adjustment.z)
    print('Current center:', center)
    
def listener():
    ros.Subscriber('armors', Armors, callback)
    ros.Subscriber('auto_aiming_adjustment', Vector3, adjustment_callback)
    ros.spin()

if __name__ == '__main__':
    ros.init_node('auto_aiming_pnp', anonymous=True)
    try:
        listener()
    except ros.ROSInterruptException:
        pass
