#!/usr/bin/env python
import rospy as ros
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
import numpy as np

count = 0

def callback(img):
    global count
    ros.loginfo('Received image. Image width: ' + str(img.width) + ' Image height: ' + str(img.height))
    im = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, -1)
    count += 1
    plt.imsave(str(count) + '.png', im)

def main():
    ros.init_node('listener', anonymous = True)
    ros.Subscriber('camera/color/image_raw', Image, callback, queue_size=100)
    ros.spin()

if __name__ == '__main__':
    main()