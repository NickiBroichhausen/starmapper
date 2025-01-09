
import numpy as np
import cv2
from cv2 import aruco
import argparse
import glob
import os.path
import click
import time


def is_opencv47():
    (major, minor, _) = cv2.__version__.split(".")
    return int(major) >= 4 and int(minor) >= 7

if is_opencv47():
    print("OpenCV >= 4.7.0 detected. Using new API.")
else:
    print("OpenCV < 4.7.0 detected. Using old API.")

parser = argparse.ArgumentParser(description='Compute camera calibration.')
parser.add_argument(dest='folder', type=str, help='Folder with *.jpg images.')
args = parser.parse_args()

CHARUCOBOARD_ROWCOUNT = 6
CHARUCOBOARD_COLCOUNT = 9
CHARUCOBOARD_SQUARE_LENGTH = 0.030
CHARUCOBOARD_MARKER_SIZE = 0.022

if is_opencv47():
    dictionary = cv2.aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
    board = cv2.aruco.CharucoBoard((CHARUCOBOARD_COLCOUNT, CHARUCOBOARD_ROWCOUNT), CHARUCOBOARD_SQUARE_LENGTH, CHARUCOBOARD_MARKER_SIZE, dictionary)
    board.setLegacyPattern(True)
    detctorparams = cv2.aruco.DetectorParameters()
    charucoparams = cv2.aruco.CharucoParameters()
    charucoparams.tryRefineMarkers = True
    detector = cv2.aruco.CharucoDetector(board, charucoparams, detctorparams)
else:
    dictionary = aruco.Dictionary_get(aruco.DICT_6X6_1000)
    board = aruco.CharucoBoard_create(
        squaresX=CHARUCOBOARD_COLCOUNT,
        squaresY=CHARUCOBOARD_ROWCOUNT,
        squareLength=0.030,
        markerLength=0.022,
        dictionary=dictionary)

images = glob.glob(args.folder + '*.jpg')

if len(images) < 3:
    print("Calibration was unsuccessful. Too few images were found.")
    exit()

image_size_left = None
corners_all_left = []
ids_all_left = []
image_left = []

# image_size_right = None
# corners_all_right = []
# ids_all_right = []
# image_right = []

for iname in images:
    print(iname)
    img = cv2.imread(iname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    left = img[0:img.shape[0], 0:int(img.shape[1])]
    leftGray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    if len(image_left) < 1:
        image_left = left.copy()

    if is_opencv47():
        charuco_corners, charuco_ids, corners, ids = detector.detectBoard(leftGray)
    else:
        corners, ids, _ = aruco.detectMarkers(
            image=leftGray,
            dictionary=dictionary)
        response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            markerCorners=corners,
            markerIds=ids,
            image=leftGray,
            board=board)
    left = aruco.drawDetectedMarkers(
        image=left, 
        corners=corners)
    if type(charuco_ids) == np.ndarray    and len(charuco_ids) > 1:
        corners_all_left.append(charuco_corners)
        ids_all_left.append(charuco_ids)
        
        left = aruco.drawDetectedCornersCharuco(
            image=left,
            charucoCorners=charuco_corners,
            charucoIds=charuco_ids)

    if not image_size_left:
        image_size_left = leftGray.shape[::-1]

    # right = img[0:img.shape[0], int(img.shape[1]/2):img.shape[1]]
    # rightGray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
    # if len(image_right) < 1:
    #     image_right = right.copy()

    # if is_opencv47():
    #     charuco_corners, charuco_ids, corners, ids = detector.detectBoard(rightGray)
    # else:
    #     corners, ids, _ = aruco.detectMarkers(
    #         image=rightGray,
    #         dictionary=dictionary)
    #     response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
    #         markerCorners=corners,
    #         markerIds=ids,
    #         image=rightGray,
    #         board=board)
    # right = aruco.drawDetectedMarkers(
    #     image=right, 
    #     corners=corners)
    # if len(charuco_ids) > 1:
    #     corners_all_right.append(charuco_corners)
    #     ids_all_right.append(charuco_ids)
        
    #     right = aruco.drawDetectedCornersCharuco(
    #         image=right,
    #         charucoCorners=charuco_corners,
    #         charucoIds=charuco_ids)

    # if not image_size_right:
    #     image_size_right = rightGray.shape[::-1]

    displayImg = cv2.hconcat([left])
    proportion = max(displayImg.shape) / 1000.0
    displayImg = cv2.resize(displayImg,
        (int(displayImg.shape[1]/proportion), int(displayImg.shape[0]/proportion)))

    cv2.imshow('Detected ChArUco markers', displayImg)
    cv2.waitKey(100)

    # #constant definitions
    # pixel_size = 3 * 10**(-6) # real pixel size from data sheat
    # T = [[-0.05977191],
    #     [-0.00023067],
    #     [ 0.00194674]]
    # camera_matrix_right = [[592.97749133,   0,         416.537874  ],
    #                         [  0,        592.875458,   301.02881231],
    #                         [  0,           0,           1.0        ]]

    # distance_z = (T[0][0] * camera_matrix_right[0][0] * pixel_size)/((corners_all_right[-1][0][0][0] - corners_all_left[-1][0][0][0]) * pixel_size)
    # print(distance_z)
    # time.sleep(2)


cv2.destroyAllWindows()

fs = None
if os.path.isfile("calibration.json"):
    if click.confirm('\nFile calibration.json exists. Overwrite?', default=True):
        fs = cv2.FileStorage("calibration.json", cv2.FILE_STORAGE_WRITE)
else:
    fs = cv2.FileStorage("calibration.json", cv2.FILE_STORAGE_WRITE)

ret, camera_matrix_left, dist_coeffs_left, rvecs, tvecs = aruco.calibrateCameraCharuco(
    charucoCorners=corners_all_left,
    charucoIds=ids_all_left,
    board=board,
    imageSize=image_size_left,
    cameraMatrix=None,
    distCoeffs=None)

tvecs_third_column = [tvec[2] for tvec in tvecs]

image_rvec_pairs = list(zip(images, rvecs))
image_tvec_third_column_pairs = list(zip(images, tvecs_third_column))

print("Image-Rvec pairs:")
for pair in image_rvec_pairs:
    print(pair)

print("\nImage-Tvec third column pairs:")
for pair in image_tvec_third_column_pairs:
    print(pair)

print("\nLeft camera:")
print(f"Error: {ret}")
print("Camera matrix:")
print(camera_matrix_left)
print("Distortion coefficients:")
print(dist_coeffs_left)

new_camera_matrix_left, roi = cv2.getOptimalNewCameraMatrix(camera_matrix_left, dist_coeffs_left, image_size_left, 1, image_size_left)
undistorted_image_left = cv2.undistort(image_left, camera_matrix_left, dist_coeffs_left, None, new_camera_matrix_left)

if fs:
    # fs.startWriteStruct('left', cv2.FileNode_MAP)
    fs.write(name='image_size',val=image_size_left)
    fs.write(name='camera_matrix',val=camera_matrix_left)
    fs.write(name='distortion_coefficients',val=dist_coeffs_left)
    fs.write(name='new_camera_matrix',val=new_camera_matrix_left)
    # fs.endWriteStruct()
    fs.release()

displayImg = cv2.vconcat([cv2.hconcat([image_left]), cv2.hconcat([undistorted_image_left])])
proportion = max(displayImg.shape) / 1000.0
displayImg = cv2.resize(displayImg, (int(displayImg.shape[1]/proportion), int(displayImg.shape[0]/proportion)))


cv2.imshow('Original and undistorted image', displayImg)
cv2.imwrite('display.png', displayImg)
