#!/usr/bin/env python
import torch
import cv2 as cv
import numpy as np
import rospy as ros
from roborts_msgs.msg import GimbalAngle
from roborts_msgs.srv import ShootCmd, ShootCmdRequest, ShootCmdResponse

class PlainPredictSpeed:
    def __init__(self):
        self.history = np.array([])
        self.weights = np.array([0.05, 0.1, 0.15, 0.3, 0.4])

    def predict(self, tar):
        self.record(tar)
        if len(self.history) <= 1:
            return 0
        if len(self.history) <= 5:
            return self.history[-1] - self.history[-2]
        v = np.array([0, 0])
        for i in range(5):
            v += (self.history[-i-1] - self.history[-i-2]) * self.weights[i]
        return v

    def record(self, tar):
        self.history.append(tar)
        if len(self.history) > 6:
            self.history = self.history[-6:]

    def reset(self, tar):
        self.history = np.array([tar])

class PredictSpeed: # Kalman Filter
    def __init__(self):
        self.kf = cv.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.kf.measurementNoiseCov = np.identity(4, dtype=np.float32) * 10 
        self.kf.processNoiseCov = np.identity(4, dtype=np.float32) * 0.01 

    def predict(self, tar): # Record position and predict
        measured = np.array([[np.float32(tar[0])], [np.float32(tar[1])]])        
        predicted = self.kf.predict()
        self.kf.correct(measured)

        predicted = [int(predicted[0]), int(predicted[1]), float(predicted[2]), float(predicted[3])]
        ros.loginfo('Discrepency x=' + str(predicted[0] - tar[0]) + ', y=' + str(predicted[1] - tar[1]))
        ros.loginfo('Predict speed x=' + str(predicted[2]) + ', y=' + str(predicted[3]))
        
        return predicted

    def record(self, tar): # Record position without predicting
        measured = np.array([[np.float32(tar[0])], [np.float32(tar[1])]])
        self.kf.correct(measured)

    def reset(self, tar): # Reset Kalman filter with initial coordinates
        self.kf = cv.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.kf.measurementNoiseCov = np.identity(2, dtype=np.float32) * 10 # Measurement noise
        self.kf.processNoiseCov = np.identity(4, dtype=np.float32) * 0.01 # Process noise
        self.kf.statePre = np.array([[tar[0]], [tar[1]], [0], [0]], dtype=np.float32)
        self.kf.statePost = np.array([[tar[0]], [tar[1]], [0], [0]], dtype=np.float32)

class ArmorDetector:
    def __init__(self, model_path='model.pt'):
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=True)

    def inference(self, img):
        robot_list = self.model(img, size=320)
        robot_list = robot_list.pandas().xyxy[0].to_numpy()
        return robot_list

class Controller:
    def __init__(self):
        self.pub = ros.Publisher('cmd_gimbal_angle', GimbalAngle, queue_size=100)
        self.gimbal_angle = GimbalAngle()
        self.gimbal_angle.yaw_mode = True
        self.gimbal_angle.pitch_mode = True
        self.client = ros.ServiceProxy("/cmd_shoot", ShootCmd)
        self.client.wait_for_service()
        self.count = 0
    
    def move(self, yaw_angle, pitch_angle, offset, log=False): # Yaw: + Left, - Right. Pitch: + Down, - Up. Offset: counters gimbal gravity
        self.gimbal_angle.yaw_angle = yaw_angle
        self.gimbal_angle.pitch_angle = pitch_angle - offset
        self.pub.publish(self.gimbal_angle)
        if log: ros.loginfo('Action: ' + 'yaw=' + str(yaw_angle) + ' pitch=' + str(pitch_angle) + ' pitch_with_offset=' + str(pitch_angle - offset))

    def shoot(self, mode, number=1): # Shoot. Mode: 0 stop, 1 once, 2 continuous. When mode=2, specify bullet number.
        req = ShootCmdRequest()
        req.mode = mode
        req.number = number
        self.count += 1
        ros.loginfo(f'Requested shoot. Count: {self.count}')
        if self.count % 2 == 0:
            response = self.client.call(req)
            ros.loginfo("Shoot service call successful? %d", response.received)

    def endshoot(self):
        req = ShootCmdRequest()
        req.mode = 0
        req.number = 0
        response = self.client.call(req)
        ros.loginfo("Shoot service call successful? %d", response.received)
