import numpy as np
import cv2
import glob

# termination criteria
n_row = 8
n_col = 6
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(n_row-1,n_col-1,0)
objp = np.zeros((n_col * n_row, 3), np.float32)
objp[:, :2] = np.mgrid[0:n_row, 0:n_col].T.reshape(-1, 2)
# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.
images = glob.glob("frames/*.jpg")
for fname in images:
    print(fname)
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (n_row, n_col), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        print("Found checked board")
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv2.drawChessboardCorners(img, (n_row, n_col), corners2, ret)
        cv2.imshow("img", img)
        cv2.waitKey(500)
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None
            )
        print(ret, mtx, dist, rvecs, tvecs)
cv2.destroyAllWindows()


