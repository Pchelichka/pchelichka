import os
import numpy as np
import math
import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Float64, Float64MultiArray
import cv2 
from ament_index_python.packages import get_package_share_directory

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
    self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_1000)
    self.aruco_params = cv2.aruco.DetectorParameters_create()
    self.publisher = self.create_publisher(Float64MultiArray, 'target', 10) 
    # self.x_publisher = self.create_publisher(String, 'target/x', 10) 
    # self.y_publisher = self.create_publisher(String, 'target/y', 10) 
    # self.z_publisher = self.create_publisher(String, 'target/z', 10) 
    # self.theta_publisher = self.cree_publisher(String, 'target/theta', 10) 
    self.timer = self.create_timer(1 / 30, self.callback)
    self.vid = cv2.VideoCapture(4) 
    self.aruco_side_length = 0.05 # meters
    self.marker_points = [
        [-self.aruco_side_length / 2, self.aruco_side_length / 2, 0],
        [self.aruco_side_length / 2, self.aruco_side_length / 2, 0],
        [self.aruco_side_length / 2, -self.aruco_side_length / 2, 0],
        [-self.aruco_side_length / 2, -self.aruco_side_length / 2, 0]
    ]
    with np.load(os.path.join(get_package_share_directory('perception'), 'calibration.npz')) as cf:
        self.mtx, self.dist = cf['mtx'], cf['dist']
    print(self.mtx, self.dist) 
  def callback(self):
    _, current_frame = self.vid.read() 
    corners, _, _ = cv2.aruco.detectMarkers(current_frame, self.aruco_dict,
        parameters=self.aruco_params)
    if corners:
        for tag in corners[0]:
            c = (0, 0)
            for corner in tag:
                c = (c[0] + corner[0], c[1] + corner[1])
            c = (int(c[0] / len(tag)), int(c[1] / len(tag)))

            # msg = String()
            # msg.data = f'{c[0]},{c[1]}'
            # self.publisher.publish(msg)
            current_frame = cv2.circle(current_frame, c, 5, (255, 0, 0), 2) 
        # ret, rvecs, tvecs = cv2.solvePnP(self.marker_points, corners, self.mtx, self.dist)
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[0], 0.18, self.mtx,
                                                                       self.dist)

        # Draw Axis
        cv2.drawFrameAxes(current_frame, self.mtx, self.dist, rvec, tvec, 0.01)  
        # if ret:
        R, _ = cv2.Rodrigues(rvec)
        x, y, z = tvec[0][0]
        theta = rotationMatrixToEulerAngles(R)[1]
        msg = Float64MultiArray()
        msg.data = [x * 100, y, z, theta]
        # print(cv2.rot2euler(rvec))
        self.publisher.publish(msg)

    current_frame = cv2.circle(current_frame, (320, 240), 5, (0, 255, 0), 2)
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

