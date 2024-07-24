import rclpy
from rclpy.node import Node
import time
from .tx import Tx
from simple_pid import PID

from std_msgs.msg import String


class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.armed = False
        self.camera_width = 640
        self.camera_height = 480
        self.roll_pid = PID(0.5, 0, 1.8, setpoint=self.camera_width / 2)
        self.throttle_pid = PID(0.14, 0.001, 0.1, setpoint=self.camera_height / 2)
        self.subscription = self.create_subscription(
            String,
            'target',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.tx = Tx()
        time.sleep(1)
        self.tx.arm()
        time.sleep(2)
        self.armed = True

    def listener_callback(self, msg):
        if self.armed:
            parsed = msg.data.strip().split(',')
            x = float(parsed[0])
            y = float(parsed[1])
            roll = 1500 - round(self.roll_pid(x))
            throttle = 1750 + round(self.throttle_pid(y))
            print(roll)
            self.tx.update(roll = roll, throttle=throttle)



def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()