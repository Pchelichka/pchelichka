import rclpy
from rclpy.node import Node
import time
from .tx import Tx
from simple_pid import PID
from std_msgs.msg import Float64MultiArray

def clip(value: int, lower: int, upper: int):
    return lower if value < lower else upper if value > upper else value

class PIDController(Node):

    def __init__(self):
        super().__init__('controller')
        self.armed = False
        self.publisher = self.create_publisher(Float64MultiArray, 'effort', 10) 
        self.roll_pid = PID(120, 4, 50, setpoint=0)
        self.pitch_pid = PID(120, 4, 50, setpoint=1)
        self.yaw_pid = PID(32, 0, 5, setpoint=0)
        self.throttle_pid = PID(20, 1.2, 50, setpoint=0)
        self.throttle_ff = 1411
        self.subscriber = self.create_subscription(
            Float64MultiArray,
            'target',
            self.listener_callback,
            10)
        self.tx = Tx(False)
        time.sleep(1)
        self.tx.arm()
        time.sleep(5)
        self.armed = True

    def listener_callback(self, msg: Float64MultiArray):
        if self.armed:
            x, y, z, theta = msg.data
            x /= 100
            roll = clip(1400 - round(self.roll_pid(x)), 1000, 2000)
            throttle = clip(self.throttle_ff + round(self.throttle_pid(y)), 1000, 2000)
            pitch = clip(1520 - round(self.pitch_pid(z)), 1000, 2000)
            # inverse yaw
            yaw = clip(1500 + round(self.yaw_pid(theta)), 1000, 2000)
            send_msg = Float64MultiArray()
            send_msg.data = [pitch, roll, yaw, throttle]
            self.publisher.publish(send_msg) 
            self.tx.update(pitch=pitch, roll=roll, yaw=yaw, throttle=throttle)

    def exit_gracefully(self):
        print('Exiting...')
        self.tx.land()
        self.tx.exit_gracefully()
            
def main(args=None):
    rclpy.init(args=args)

    controller = PIDController()

    rclpy.spin(controller)

    PIDController.exit_gracefully()
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()