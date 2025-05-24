import os
import time
import numpy as np
import math
import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Float64MultiArray
import cv2 
from ament_index_python.packages import get_package_share_directory
import argparse

 # Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))
    
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    
    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])
 
class Camera(Node):
  def __init__(self):
    super().__init__('image_subscriber')
    print('Version')
    print(cv2.__version__)
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['analog', 'digital'], required=True,
                        help='Choose the input type: analog (OpenCV webcam) or digital (custom C++ stream)')
    args = parser.parse_args()
    self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    self.aruco_params = cv2.aruco.DetectorParameters()
    self.publisher = self.create_publisher(Float64MultiArray, 'target', 10) 
    self.timer = self.create_timer(1 / 30, self.callback)
    if args.mode == 'analog':
        self.vid = cv2.VideoCapture(4) 
        self.WIDTH = 640
        self.HEIGHT = 480
    elif args.mode == 'digital':
        # self.vid = cv2.VideoCapture('fdsrc ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false', cv2.CAP_GSTREAMER) 
        self.vid = cv2.VideoCapture('filesrc location=/tmp/video_pipe ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false', cv2.CAP_GSTREAMER) 
        # slow pipeline
        # self.vid = cv2.VideoCapture('fdsrc ! decodebin ! videoconvert ! appsink', cv2.CAP_GSTREAMER)

        self.vid.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.WIDTH = 960
        self.HEIGHT = 720

    self.aruco_side_length = 0.1 # meters
    self.marker_points = [
        [-self.aruco_side_length / 2, self.aruco_side_length / 2, 0],
        [self.aruco_side_length / 2, self.aruco_side_length / 2, 0],
        [self.aruco_side_length / 2, -self.aruco_side_length / 2, 0],
        [-self.aruco_side_length / 2, -self.aruco_side_length / 2, 0]
    ]
    self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
    with np.load(os.path.join(get_package_share_directory('perception'), 'calibration.npz' if args.mode == 'analog' else 'calibration_dji.npz')) as cf:
        self.mtx, self.dist = cf['mtx'], cf['dist']
    print(self.mtx, self.dist) 
  def callback(self):
    ret, current_frame = self.vid.read() 
    if not ret:
       return
    corners, _, _ = self.aruco_detector.detectMarkers(current_frame)
    if corners:
        # for tag in corners[0]:
        #     c = (0, 0)
        #     for corner in tag:
        #         c = (c[0] + corner[0], c[1] + corner[1])
        #     c = (int(c[0] / len(tag)), int(c[1] / len(tag)))

            # msg = String()
            # msg.data = f'{c[0]},{c[1]}'
            # self.publisher.publish(msg)
            # current_frame = cv2.circle(current_frame, c, 5, (255, 0, 0), 2) 
        # ret, rvecs, tvecs = cv2.solvePnP(self.marker_points, corners, self.mtx, self.dist)
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[0], self.aruco_side_length, self.mtx,
                                                                       self.dist)

        # Draw Axis
        cv2.drawFrameAxes(current_frame, self.mtx, self.dist, rvec, tvec, 0.1)  
        R, _ = cv2.Rodrigues(rvec)
        x, y, z = tvec[0][0]
        theta = rotationMatrixToEulerAngles(R)[1]
        msg = Float64MultiArray()
        msg.data = [x * 100, y * 100, z * 100, theta, time.time()]
        # print(cv2.rot2euler(rvec))
        self.publisher.publish(msg)

    current_frame = cv2.circle(current_frame, (self.WIDTH//2, self.HEIGHT//2), 5, (0, 255, 0), 2)
    cv2.imshow("camera", current_frame)
    
    cv2.waitKey(1)
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = Camera()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()

