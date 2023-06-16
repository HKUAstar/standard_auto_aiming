#!/usr/bin/env python3
import numpy as np
import cv2 as cv
import rospy as ros

class PredictSpeed: # Kalman Filter
    def __init__(self):
        self.kf = cv.KalmanFilter(6, 3)
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]], dtype=np.float32)
        self.kf.transitionMatrix = np.array([
            [1, 0, 0, 1, 0, 0],
            [0, 1, 0, 0, 1, 0],
            [0, 0, 1, 0, 0, 1],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0], 
            [0, 0, 0, 0, 0, 1]], dtype=np.float32)
        self.kf.measurementNoiseCov = np.identity(3, dtype=np.float32) * 1 
        self.kf.processNoiseCov = np.identity(6, dtype=np.float32) * 0.01 

    def predict(self, tar): # Record position and predict
        measured = np.array(tar, dtype=np.float32).reshape(-1)
        predicted = self.kf.predict()
        self.kf.correct(measured)

        #ros.loginfo('Predict difference x=' + str(predicted[0] - tar[0]) + ', y=' + str(predicted[1] - tar[1]))
        #ros.loginfo('Predict speed x=' + str(predicted[2]) + ', y=' + str(predicted[3]))
        
        return predicted.reshape(-1)

    def record(self, tar): # Record position without predicting
        measured = np.array(tar, dtype=np.float32).reshape(-1)
        self.kf.correct(measured)

    def reset(self, tar): # Reset Kalman filter with initial coordinates
        self.kf = cv.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.kf.measurementNoiseCov = np.identity(2, dtype=np.float32) * 10 # Measurement noise
        self.kf.processNoiseCov = np.identity(4, dtype=np.float32) * 0.01 # Process noise
        self.kf.statePre = np.array([[tar[0]], [tar[1]], [0], [0]], dtype=np.float32)
        self.kf.statePost = np.array([[tar[0]], [tar[1]], [0], [0]], dtype=np.float32)

def body_to_cam(p_world, angles, translation):
    # angles: rotate about z-axis, x-axis, y-axis of camera
    # z-axis: pointing to the front, x-axis: pointing right, y-axis: pointing down
    mat = np.array([
        [
            [np.cos(angles[0]), -np.sin(angles[0]), 0],
            [np.sin(angles[0]), np.cos(angles[0]), 0],
            [0, 0, 1]
        ],
        [
            [1, 0, 0],
            [0, np.cos(angles[1]), np.sin(angles[1])],
            [0, -np.sin(angles[1]), np.cos(angles[1])]
        ],
        [
            [np.cos(angles[2]), 0, -np.sin(angles[2])],
            [0, 1, 0],
            [np.sin(angles[2]), 0, np.cos(angles[2])]
        ]])
    rotation = mat[2] @ mat[1] @ mat[0]
    return (rotation @ (p_world - translation))[:3]

def cam_to_body(p_cam, angles, translation):
    # angles: rotate about z-axis, x-axis, y-axis of camera
    # z-axis: pointing to the front, x-axis: pointing right, y-axis: pointing down
    mat = np.array([
        [
            [np.cos(angles[0]), -np.sin(angles[0]), 0],
            [np.sin(angles[0]), np.cos(angles[0]), 0],
            [0, 0, 1]
        ],
        [
            [1, 0, 0],
            [0, np.cos(angles[1]), np.sin(angles[1])],
            [0, -np.sin(angles[1]), np.cos(angles[1])]
        ],
        [
            [np.cos(angles[2]), 0, -np.sin(angles[2])],
            [0, 1, 0],
            [np.sin(angles[2]), 0, np.cos(angles[2])]
        ]])
    rotation = mat[2] @ mat[1] @ mat[0]
    return (np.linalg.inv(rotation) @ p_cam) + translation

def pixel_to_cam(p_img, camera_mtx, camera_dist, id=3):
    if id == 1 or id == 7:
        armor = np.array([
            [0.115, 0.0635, 0.],
            [0.115, -0.0635, 0.],
            [-0.115, -0.0635, 0.],
            [-0.115, 0.0635, 0.]])
    else:
        armor = np.array([
            [0.0675, 0.0625, 0.],
            [0.0675, -0.0625, 0.],
            [-0.0675, -0.0625, 0.],
            [-0.0675, 0.0625, 0.]]) # to be changed with slope
    ret, rvec, tvec = cv.solvePnP(armor, p_img, camera_mtx, camera_dist)
    return rvec, tvec