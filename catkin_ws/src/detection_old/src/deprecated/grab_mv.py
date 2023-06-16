#!/usr/bin/env python
#coding=utf-8
import rospy as ros
import ros_numpy
from sensor_msgs.msg import Image
import cv2 as cv
import mvsdk
import numpy as np

def main():
    ros.init_node('talker', anonymous=True)
    pub = ros.Publisher('/image', Image, queue_size=100)

    DevList = mvsdk.CameraEnumerateDevice()
    nDev = len(DevList)
    if nDev < 1:
        print('No camera found!')
        return
    for i, DevInfo in enumerate(DevList):
        print(f'{i}: {DevInfo.GetFriendlyName()} {DevInfo.GetPortType()}')
    
    print(DevInfo)
    hCamera = 0
    try:
        hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
    except mvsdk.CameraException as e:
        print("CameraInit Failed({}): {}".format(e.error_code, e.message) )
        return

    cap = mvsdk.CameraGetCapability(hCamera)

    mvsdk.CameraSetTriggerMode(hCamera, 0)

    mvsdk.CameraSetAeState(hCamera, 0)
    mvsdk.CameraSetExposureTime(hCamera, 9 * 1000)

    mvsdk.CameraPlay(hCamera)

    FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * 3

    pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

    count = 0

    while (cv.waitKey(1) & 0xFF) != ord('q'):
        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 1)
            mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3) )

            #frame = cv.resize(frame, (640, 480), interpolation = cv.INTER_LINEAR)
            #frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
            
            #cv.imshow("Press q to end", frame)

            count += 1
            cv.imwrite('Image.png', frame)
            #img = ros_numpy.msgify(Image, frame)
            #pub.publish(img)
            if count % 108 == 0:
                ros.loginfo(f'Sent {count} images.')

        except mvsdk.CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message) )
    
    mvsdk.CameraUnInit(hCamera)
    mvsdk.CameraAlignFree(pFrameBuffer)

if __name__ == '__main__':
    try:
        main()
    finally:
        cv.destroyAllWindows()