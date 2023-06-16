import numpy as np
import cv2 as cv
import glob

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

obj_points = np.zeros((10 * 7, 3), dtype=np.float32)
obj_points[:, :2] = np.mgrid[0:7, 0:10].T.reshape(-1, 2) * 0.025

obj_points_3d = []
img_points = []

image_list = glob.glob("pictures/Image_*.bmp") # to be changed with camera

for filename in image_list:
    img = cv.imread(filename)
    gray = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
    img_shape = gray.shape[::-1]
    ret, corners = cv.findChessboardCorners(gray, (7, 10), None)

    if ret == True:
        obj_points_3d.append(obj_points)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        img_points.append(corners2)
        cv.drawChessboardCorners(img, (7, 10), corners2, ret)
        cv.imwrite("pictures/result_" + filename, img)

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(obj_points_3d, img_points, img_shape, None, None)

print("Camera matrix")
print(mtx)
print("Distortion matrix")
print(dist)

with open('camera.npy', 'wb') as file:
    np.save(file, mtx)
    np.save(file, dist)
    print('Successfully saved camera matrix')
