#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include "hikrobot_camera.hpp"
using namespace cv;
using namespace std;

int main()
{
    Mat src;
    camera::Camera image_capture;
    int cnt = 0;
    while (true)
    {
        image_capture.ReadImg(src);
        if (src.empty())
            continue;
        cout << "Read image " << ++cnt << endl;
    }
}

/*Compile with: g++ test_camera.cpp -o test_camera -I /opt/MVS/include -I /home/astar/opencv_install/build/ -L /home/astar/opencv_install/build/lib -L /opt/MVS/lib/64 -lMvCameraControl -lMVGigEVisionSDK -lMVRender -lMvUsb3vTL -lpthread -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_stitching*/