#!/usr/bin/env python3
import time
import cv2 as cv
import numpy as np
import rospy as ros
from sensor_msgs.msg import Image

cnt = 0
t0=time.time()

def callback(img):
    global cnt
    cnt += 1
    print(cnt / (time.time()-t0))
    # img = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, -1)
    # cv.imwrite('foo.png', img)

ros.init_node('sss')
ros.Subscriber('/hikrobot_camera/rgb', Image, callback) # get camera image
ros.spin()
