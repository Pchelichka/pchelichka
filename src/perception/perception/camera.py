import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from std_msgs.msg import String
from cv_bridge import CvBridge 
import cv2 

 
class ImageSubscriber(Node):
  def __init__(self):
    super().__init__('image_subscriber')
    self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_1000)
    self.arucoParams = cv2.aruco.DetectorParameters_create()
    self.publisher = self.create_publisher(String, 'target', 10) 
    self.subscription = self.create_subscription(
      Image, 
      '/camera/image_raw', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):

    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    corners, _, _ = cv2.aruco.detectMarkers(current_frame, self.arucoDict,
        parameters=self.arucoParams)
    if corners:
        for tag in corners[0]:
            c = (0, 0)
            for corner in tag:
                c = (c[0] + corner[0], c[1] + corner[1])
            c = (int(c[0] / len(tag)), int(c[1] / len(tag)))

            msg = String()
            msg.data = f'{c[0]},{c[1]}'
            self.publisher.publish(msg)
            current_frame = cv2.circle(current_frame, c, 5, (255, 0, 0), 2) 
    current_frame = cv2.circle(current_frame, (320, 240), 5, (0, 255, 0), 2)
    cv2.imshow("camera", current_frame)
    
    cv2.waitKey(1)
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
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
