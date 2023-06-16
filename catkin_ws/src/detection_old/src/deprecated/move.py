#!/usr/bin/env python
import rospy as ros
from roborts_msgs.msg import GimbalAngle
from math import pi

class Controller:
    def __init__(self):
        ros.init_node('play_play_node')
        pub = ros.Publisher('cmd_gimbal_angle', GimbalAngle, queue_size=100)
    
    def move(self, yaw_angle, pitch_angle):
        gimbal_angle = GimbalAngle()
        gimbal_angle.yaw_angle = yaw_angle
        gimbal_angle.pitch_angle = pitch_angle
        self.pub.publish(gimbal_angle)

    

def main():
    sleep_time = .1 * 1e-6
    ros.init_node('play_play_node')
    pub = ros.Publisher('cmd_gimbal_angle', GimbalAngle, queue_size=100)
    c = 0
    gimbal_angle = GimbalAngle()
    gimbal_angle.yaw_mode = True
    gimbal_angle.pitch_mode = True
    x = 0.01
    while not ros.is_shutdown():
        c += 1
        if c % 1000 == 0:
            if x > 0:
                x = -0.035
            else:
                x = 0.01
        gimbal_angle.yaw_angle = x
        gimbal_angle.pitch_angle = 0
        ros.loginfo(str(x))
        pub.publish(gimbal_angle)
        ros.sleep(sleep_time)

if __name__ == '__main__':
    main()