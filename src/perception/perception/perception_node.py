import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class PerceptionNode(Node):

  def __init__(self):
    super().__init__('perception_node')

    self.cameraSubscriber = self.create_subscription(
        String,
        'camera',
        self.camera_callback,
        10
    )

    self.trashDetectionPublisher = self.create_publisher(
        String,
        'trash_detection',
        10
    )

  def camera_callback(self, message):
    msg = String()
    """
    In practical cases, we process images to understand if trash is present
    """
    if (message.data == "trash"):
      msg.data = "I see trash"
    else:
      msg.data = "I see nothing"
    self.trashDetectionPublisher.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
  rclpy.init(args=args)

  perceptionNode = PerceptionNode()

  rclpy.spin(perceptionNode)

  """
  Destroy the node explicitly
  (optional - otherwise it will be done automatically
  when the garbage collector destroys the node object)
  """
  perceptionNode.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
