import os
import numpy as np
import cv2
import time

# Create arrays you'll use to store object points and image points from all images processed
objpoints = [] # 3D point in real world space where chess squares are
imgpoints = [] # 2D point in image plane, determined by CV2

# Chessboard variables
CHESSBOARD_CORNERS_ROWCOUNT = 9
CHESSBOARD_CORNERS_COLCOUNT = 6

# Theoretical object points for the chessboard we're calibrating against,
# These will come out like: 
#     (0, 0, 0), (1, 0, 0), ..., 
#     (CHESSBOARD_CORNERS_ROWCOUNT-1, CHESSBOARD_CORNERS_COLCOUNT-1, 0)
# Note that the Z value for all stays at 0, as this is a printed out 2D image
# And also that the max point is -1 of the max because we're zero-indexing
# The following line generates all the tuples needed at (0, 0, 0)
objp = np.zeros((1, CHESSBOARD_CORNERS_ROWCOUNT*CHESSBOARD_CORNERS_COLCOUNT,3), np.float32)
# The following line fills the tuples just generated with their values (0, 0, 0), (1, 0, 0), ...
objp[0,:,:2] = np.mgrid[0:CHESSBOARD_CORNERS_ROWCOUNT,0:CHESSBOARD_CORNERS_COLCOUNT].T.reshape(-1, 2)

# vid = cv2.VideoCapture(4) 
# vid = cv2.VideoCapture('fdsrc ! decodebin ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
# vid = cv2.VideoCapture('filesrc location=/tmp/video_pipe ! decodebin ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false', cv2.CAP_GSTREAMER) 
vid = cv2.VideoCapture('udpsrc port=5600 ! application/x-rtp ! rtph265depay ! h265parse ! decodebin ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false', cv2.CAP_GSTREAMER) 
imageSize = None
time.sleep(2)
start_time = time.time()
while True:
    # img = cv2.imread(fname)
    ret, img = vid.read()
    if not ret:
     continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (CHESSBOARD_CORNERS_ROWCOUNT,CHESSBOARD_CORNERS_COLCOUNT), cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    # If found, add object points, image points (after refining them)
    if ret == True:        
        # Add the points in 3D that we just discovered
        objpoints.append(objp)
        
        # Enhance corner accuracy with cornerSubPix
        corners_acc = cv2.cornerSubPix(
                image=gray, 
                corners=corners, 
                winSize=(11, 11), 
                zeroZone=(-1, -1),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)) # Last parameter is about termination critera
        imgpoints.append(corners_acc)

        # wf our image size is unknown, set it now
        if not imageSize:
            imageSize = gray.shape[::-1]
    
        # Draw the corners to a new image to show whoever is performing the calibration
        # that the board was properly detected
        img = cv2.drawChessboardCorners(img, (CHESSBOARD_CORNERS_ROWCOUNT, CHESSBOARD_CORNERS_COLCOUNT), corners_acc, ret)
    cv2.imshow('Chessboard', img)
    cv2.waitKey(500)
    if time.time() - start_time > 60:
        break

# Destroy any open CV windows
cv2.destroyAllWindows()
print('Recording done.')

# Make sure we were able to calibrate on at least one chessboard by checking
# if we ever determined the image size
if not imageSize:
    # Calibration failed because we didn't see any chessboards of the PatternSize used
    print("Calibration was unsuccessful. We couldn't detect chessboards in any of the images supplied. Try changing the patternSize passed into findChessboardCorners(), or try different pictures of chessboards.")
    # Exit for failure
    exit()

# Now that we've seen all of our images, perform the camera calibration
# based on the set of points we've discovered
cameraMatrix = np.zeros((3, 3))
distCoeffs = np.zeros((4, 1))
rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(len(imgpoints))]
tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(len(imgpoints))]
calibration, _, _, _, _ = cv2.fisheye.calibrate(
        objectPoints=objpoints,
        imagePoints=imgpoints,
        image_size=imageSize,
        K=cameraMatrix,
        D=distCoeffs,
		rvecs=rvecs,
		tvecs=tvecs,
        flags=cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_FIX_SKEW,
        criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6))
    
# Print matrix and distortion coefficient to the console
print(cameraMatrix)
print(distCoeffs)
# np.savez(os.path.join(os.path.dirname(__file__), '../resource/calibration.npz'), mtx=cameraMatrix, dist=distCoeffs)
np.savez(os.path.join(os.path.dirname(__file__), '../resource/calibration_dji.npz'), mtx=cameraMatrix, dist=distCoeffs)