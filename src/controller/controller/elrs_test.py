import rclpy
from rclpy.node import Node
import signal
import time

import rclpy.signals
from .tx import Tx, Mode
from std_msgs.msg import Int32MultiArray

def clip(value: int, lower: int, upper: int):
    return lower if value < lower else upper if value > upper else value

class ELRSTest(Node):

    def __init__(self):
        super().__init__('controller')
        self.tx = Tx(mode=Mode.ELRS, publisher=self.create_publisher(Int32MultiArray, 'rc_channels', 10))
        time.sleep(1)
        self.tx.arm()
        print('Arming...')
        self.armed = True
        self.armed_time = time.time()

    def exit_gracefully(self):
        print('Exiting...')
        self.tx.exit_gracefully()
            
def main(args=None):
    rclpy.init(args=args, signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    controller = None
    try:
        controller = ELRSTest()
        while rclpy.ok():
            rclpy.spin_once(controller, timeout_sec=0.01)
    except KeyboardInterrupt:
        print('Emergency stop')
        if controller is not None:
            controller.exit_gracefully()
    finally:
        if controller is not None:
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()