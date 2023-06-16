#!/usr/bin/env python3
import rospy as ros
from roborts_msgs.msg import GimbalAngle
from roborts_msgs.srv import ShootCmd, ShootCmdRequest, ShootCmdResponse

cnt = 0

class Controller:
    def __init__(self):
        self.pub = ros.Publisher('cmd_gimbal_angle', GimbalAngle, queue_size=100)
        self.gimbal_angle = GimbalAngle()
        self.gimbal_angle.yaw_mode = True
        self.gimbal_angle.pitch_mode = True
        #self.client = ros.ServiceProxy("/cmd_shoot", ShootCmd)
        #self.client.wait_for_service()
        self.count = 0
    
    def move_gimbal(self, yaw_angle, pitch_angle, pitch_offset): # Yaw: + Left, - Right. Pitch: + Down, - Up. Offset: counters gimbal gravity
        global cnt
        cnt += 1
        yaw_angle = yaw_angle * 1.0
        pitch_angle = pitch_angle * 0.1
        self.gimbal_angle.yaw_angle = yaw_angle
        self.gimbal_angle.pitch_angle = pitch_angle - pitch_offset
        
        self.pub.publish(self.gimbal_angle)
        ros.loginfo('Action: ' + 'yaw=' + str(yaw_angle) + ' pitch=' + str(pitch_angle) + ' pitch_with_offset=' + str(pitch_angle - pitch_offset))

    def shoot(self, mode, number=1): # Shoot. Mode: 0 stop, 1 once, 2 continuous. When mode=2, specify bullet number.
        req = ShootCmdRequest()
        req.mode = mode
        req.number = number
        self.count += 1
        ros.loginfo(f'Requested shoot. Count: {self.count}')
        if self.count % 2 == 0:
            pass
            #response = self.client.call(req)
            #ros.loginfo("Shoot service call successful? %d", response.received)

    def endshoot(self):
        req = ShootCmdRequest()
        req.mode = 0
        req.number = 0
        #response = self.client.call(req)
        #ros.loginfo("Shoot service call successful? %d", response.received)
