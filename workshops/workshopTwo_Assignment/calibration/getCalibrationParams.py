import numpy as np
import cv2
import glob
import pickle

N_ROWS = 10
N_COLS = 7
BOX_SIZE = 35e-3

# termination criteria
chessboard_size = (N_ROWS - 1, N_COLS - 1)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(n_row-1,n_col-1,0)
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2) * BOX_SIZE

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

# Calibration results storage
ret_all, mtx_all, dist_all, rvecs_all, tvecs_all = [], [], [], [], []

images = glob.glob("frames/*.jpg")
for fname in images:
    print(fname)
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        print("Found chessboard")
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        img = cv2.resize(img, (960, 540))
        cv2.imshow("output", img)  
        cv2.waitKey(100)
cv2.destroyAllWindows()
# Perform camera calibration
ret_val, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("Error in projection: "+ str(ret_val)) 
print("Camera matrix: " + str(mtx)) 
print("Distortion coefficients:" + str(dist)) 
print("Rotation vector: " + str(rvecs)) 
print("Translation vector: " + str(tvecs))

pickle.dump((mtx, dist), open("calibration.pkl", "wb"))
