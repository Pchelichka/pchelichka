import os
import sys
import time
import math
import threading
import queue
import argparse
import signal
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory

VISION_FPS = 90
FRAME_QUEUE_SIZE = 2  # only keep the newest frame

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    assert isRotationMatrix(R)
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])

class Camera(Node):
    def __init__(self, args):
        super().__init__('image_subscriber')

        # --- Setup camera ---
        self.mode = args.source
        cal_file = {
            'analog': 'calibration.npz',
            'dji': 'calibration_dji.npz',
            'openipc': 'calibration_openipc.npz'
        }[self.mode]

        with np.load(os.path.join(get_package_share_directory('perception'), cal_file)) as cf:
            self.mtx, self.dist = cf['mtx'], cf['dist']
            self.zero_dist = np.zeros_like(self.dist)

        if self.mode == 'openipc':
            pipeline = (
                'udpsrc port=5600 ! application/x-rtp ! rtph265depay ! h265parse ! decodebin ! '
                'videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false'
            )
            self.WIDTH, self.HEIGHT = 1280, 720
        elif self.mode == 'analog':
            pipeline = 4
            self.WIDTH, self.HEIGHT = 640, 480
        else:
            pipeline = (
                'filesrc location=/tmp/video_pipe ! decodebin ! videoconvert ! '
                'video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false'
            )
            self.WIDTH, self.HEIGHT = 960, 720

        self.vid = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER if isinstance(pipeline, str) else 0)
        self.vid.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # --- Precompute undistortion map ---
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            self.mtx, self.dist, np.eye(3), self.mtx, (self.WIDTH, self.HEIGHT), cv2.CV_16SC2
        )

        # --- ArUco setup ---
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.aruco_side_length = 0.1
        self.marker_points = np.array([
            [-self.aruco_side_length / 2,  self.aruco_side_length / 2, 0],
            [ self.aruco_side_length / 2,  self.aruco_side_length / 2, 0],
            [ self.aruco_side_length / 2, -self.aruco_side_length / 2, 0],
            [-self.aruco_side_length / 2, -self.aruco_side_length / 2, 0]
        ])

        # --- ROS setup ---
        self.publisher = self.create_publisher(Float64MultiArray, 'target', 10)
        self.timer = self.create_timer(1 / VISION_FPS, self.callback)

        # --- Threaded frame capture ---
        self.frame_queue = queue.Queue(maxsize=FRAME_QUEUE_SIZE)
        self.running = True
        self.capture_thread = threading.Thread(target=self._frame_reader, daemon=True)
        self.capture_thread.start()

        self.last_t = time.time()
        self.get_logger().info("Threaded camera initialized")

    def _frame_reader(self):
        """Continuously read frames and store latest in queue (non-blocking)."""
        while self.running:
            ret, frame = self.vid.read()
            if not ret:
                time.sleep(0.001)
                continue
            frame = cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
            if self.frame_queue.full():
                try:
                    _ = self.frame_queue.get_nowait()  # discard oldest
                except queue.Empty:
                    pass
            self.frame_queue.put_nowait(frame)

    def callback(self):
        """Non-blocking processing of latest frame."""
        try:
            frame = self.frame_queue.get_nowait()
        except queue.Empty:
            return

        # --- ArUco detection ---
        corners, ids, _ = self.aruco_detector.detectMarkers(frame)
        if corners:
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[0], self.aruco_side_length, self.mtx, self.zero_dist
            )
            cv2.drawFrameAxes(frame, self.mtx, self.zero_dist, rvec, tvec, 0.1)  
            R, _ = cv2.Rodrigues(rvec)
            x, y, z = tvec[0][0]
            theta = rotationMatrixToEulerAngles(R)[1]
            msg = Float64MultiArray()
            msg.data = [x * 100, y * 100, z * 100, theta, time.time()]
            self.publisher.publish(msg)

        # --- Display (optional) ---
        cv2.imshow("camera", frame)
        cv2.waitKey(1)

        # --- FPS measurement ---
        now = time.time()
        fps = 1.0 / (now - self.last_t)
        self.last_t = now
        self.get_logger().debug(f"Vision loop FPS: {fps:.1f}")

    def destroy_node(self):
        """Graceful shutdown."""
        self.running = False
        self.capture_thread.join(timeout=1.0)
        self.vid.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    if '--ros-args' in sys.argv:
        idx = sys.argv.index('--ros-args')
        user_args = sys.argv[1:idx]
    else:
        user_args = sys.argv[1:]

    parser = argparse.ArgumentParser()
    parser.add_argument('--source', choices=['analog', 'dji', 'openipc'], required=True)
    parsed_args = parser.parse_args(user_args)

    rclpy.init(args=args)
    node = Camera(parsed_args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
