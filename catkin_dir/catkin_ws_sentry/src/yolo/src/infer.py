#! /home/sentry/anaconda3/envs/sentry/bin/python3
import os
import cv2
import torch
import time
from ultralytics import YOLO
from ultralytics.yolo.utils.plotting import Annotator

path = "/home/sentry/catkin_ws/src/yolo/src/models/best2.pt"

print("cuda status: ", torch.cuda.is_available())
model = YOLO(path)
img = cv2.imread("./img.bmp")
img = cv2.resize(img, (640, 640))
results = model(img)
print(len(results))
print(results[0].boxes.cls)