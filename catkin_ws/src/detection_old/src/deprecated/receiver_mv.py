#!/usr/bin/env python
import rospy as ros
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
import numpy as np

count = 0

def callback(img):
    global count
    count += 1
    if count % 30 == 0:
        ros.loginfo(f'Received {count} images. Width: {img.width}, Height: {img.height}')
    im = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, -1)
    plt.imsave("Image.png", im)

def main():
    ros.init_node('listener', anonymous=True)
    ros.Subscriber('/hikrobot_camera/rgb', Image, callback, queue_size=100)
    ros.spin()

if __name__ == '__main__':
    main()