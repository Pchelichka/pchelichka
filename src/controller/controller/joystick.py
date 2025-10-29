import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import pygame
import signal

from .util import channel_value, sig_handler

class Joystick(Node):
    def __init__(self):
        super().__init__('joystick')
        pygame.joystick.init()
        self.clock = pygame.time.Clock()
        self.FPS = 30
        self.running = True
        self.joystick = None
        self.publisher = self.create_publisher(Int32MultiArray, 'manual_rc_chan', 10) 
        self.timer = self.create_timer(1 / self.FPS, self.callback)
    def callback(self):
        msg = Int32MultiArray()
        msg.data = []
        if not self.running:
            self.exit_gracefully()
            return
        self.clock.tick(self.FPS)
        if self.joystick:
            self.joystick.quit()
            if self.init_joystick():
                for i in range(7):
                    chan_val = channel_value((self.joystick.get_axis(i) / 2 + 1.5) * 1000)
                    msg.data.append(chan_val)
                    # self.get_logger().info(f'{i}: {chan_val}')
        else:
            self.init_joystick()
        self.publisher.publish(msg)
    def init_joystick(self):
        try:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            return True
        except:
            self.get_logger().warn('No joystick detected.')
            self.joystick = None
            return False
        
    def exit_gracefully(self):
        self.get_logger().info('Exiting...')
        self.timer.cancel()
        self.timer = None
        

def main(args=None):
    signal.signal(signal.SIGTERM, sig_handler)
    signal.signal(signal.SIGINT, sig_handler)
    rclpy.init(args=args, signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)

    joystick = None
    try:
        joystick = Joystick()
        while rclpy.ok():
            rclpy.spin_once(joystick, timeout_sec=0.01)
    except (KeyboardInterrupt, SystemExit):
        if joystick is not None:
            joystick.get_logger().fatal('Emergency stop!')
            joystick.exit_gracefully()
        else:
            print('Emergency stop!')
    finally:
        if joystick is not None:
            joystick.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()