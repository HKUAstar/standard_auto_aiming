#! /home/sentry/anaconda3/envs/sentry/bin/python3
import rospy
import os
import cv2
import torch
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from ultralytics.yolo.utils.plotting import Annotator

path = "/home/sentry/catkin_ws/src/yolo/src/models/best2.pt"

model = YOLO(path)
count = 0

def callback(msg):
    # Convert the ROS Image message to a OpenCV image
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")

    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    results = model.predict(img, device="cuda:0")
    global count
    count += 1
    for r in results:
        annotator = Annotator(frame)
        boxes = r.boxes
        for box in boxes:
            b = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format
            c = box.cls
            rospy.loginfo(f"写出的数据: {model.names[int(c)]}")
            annotator.box_label(b, model.names[int(c)])
    frame = annotator.result()
    # cv2.imshow('YOLO V8 Detection', frame)
    msg = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
    
    global start_time, frame_count
    if time.time() - start_time > 1.0:
        start_time = time.time()
        frame_count = count
        count = 0
    rospy.loginfo(f"frame per second: {frame_count}")
    
    pub.publish(msg)
    return img


if __name__ == "__main__":
    print("cuda status: ", torch.cuda.is_available())

    global start_time
    start_time = None
    start_time = time.time()

    rospy.init_node("detection", disable_signals=True)
    sub = rospy.Subscriber("/img", Image, callback, queue_size=10)
    pub = rospy.Publisher("/annotated_img", Image, queue_size=10)
    rospy.spin()