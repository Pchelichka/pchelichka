import sys
import rclpy
from rclpy.node import Node
import time
from .util import channel_value, sig_handler
from .tx import Tx, Mode
from .pid import PID, CascadePID
from .configuration import Config, ConfigType
from .commands import Position
from .savgol_filtered_data import SavgolFilteredData
from std_msgs.msg import Float64MultiArray, Int32MultiArray
import math
import argparse
import signal
from telemetry_interfaces.srv import GetAttitude, GetPitch, GetRoll, GetYaw, GetBatteryVoltage

CONTROL_THROTTLE = 1500
VISION_FPS = 90
UPDATE_RATE = VISION_FPS

class PIDController(Node):

    def __init__(self, tx_mode = Mode.ELRS):
        super().__init__('controller')
        self.tx_mode = tx_mode
        self._sequence_started = False
        self._taking_off = False
        self.effort_publisher = self.create_publisher(Float64MultiArray, 'effort', 10) 
        self.velocity_publisher = self.create_publisher(Float64MultiArray, 'velocity', 10) 
        self.velocity_target_publisher = self.create_publisher(Float64MultiArray, 'velocity_target', 10) 
        self.acc_publisher = self.create_publisher(Float64MultiArray, 'acc', 10) 
        self.acc_target_publisher = self.create_publisher(Float64MultiArray, 'acc_target', 10) 
        self.config = Config(type = ConfigType.SERVER)
        self.roll_pid = CascadePID(3, 0, 
                                   self.config.roll_pid_constants)   
        self.pitch_pid = CascadePID(3, 0, 
                                    self.config.pitch_pid_constants)   
        self.yaw_pid = PID(self.config.yaw_pid_constants[0][0], self.config.yaw_pid_constants[0][1], self.config.yaw_pid_constants[0][2], setpoint=0)
        self.throttle_pid = CascadePID(3, 0, 
                                       self.config.throttle_pid_constants)  
        # disable roll and pitch adjustments for takeoff 
        self.roll_pid.set_auto_mode(False)
        self.pitch_pid.set_auto_mode(False)
        self.throttle_pid.set_auto_mode(False)
        self.yaw_pid.set_auto_mode(False)
        self.throttle_ff = self.config.throttle_ff
        self.roll_ff = self.config.roll_ff
        self.pitch_ff = self.config.pitch_ff
        self.yaw_ff = self.config.yaw_ff
        self.sequence = self.config.sequence
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.theta = 0
        self.prev_theta = None
        self.prev_unfiltered_theta = None
        self.last_camera_recv_t = None
        self.battery_voltage = 0
        self.x = SavgolFilteredData()
        self.y = SavgolFilteredData()
        self.z = SavgolFilteredData()
        self.theta = 0
        self.attitude_client = self.create_client(GetAttitude, '/get_attitude')
        self.pitch_client = self.create_client(GetPitch, '/get_pitch')
        self.roll_client = self.create_client(GetRoll, '/get_roll')
        self.yaw_client = self.create_client(GetYaw, '/get_yaw')
        self.battery_voltage_client = self.create_client(GetBatteryVoltage, '/get_battery_voltage')

        self.get_logger().info('Checking services...') 
        while not self.attitude_client.wait_for_service(timeout_sec=1.0) and not (self.pitch_client.wait_for_service(timeout_sec=1.0) and self.roll_client.wait_for_service(timeout_sec=1.0) and self.yaw_client.wait_for_service(timeout_sec=1.0)):
            self.get_logger().info('Attitude service(s) not available, waiting...')
        while not self.battery_voltage_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Battery voltage service not available, waiting again...')
        self.timer_action = self.create_timer(0.01, self.call_telemetry_services)
        self.control_timer = self.create_timer(1 / UPDATE_RATE, self.control)
        self.subscriber = self.create_subscription(
            Float64MultiArray,
            'target',
            self.listener_callback,
            10)
        if any(self.config.manual.values()):
            self.manual_rc_channels = []
            self.joystick_subscriber = self.create_subscription(
                Int32MultiArray,
                'manual_rc_chan',
                self.joystick_listener_callback,
                10)
        self.tx = Tx(mode=self.tx_mode, publisher=self.create_publisher(Int32MultiArray, 'rc_channels', 10) if self.tx_mode == Mode.ELRS else None)
        # Wait for tx initialization
        time.sleep(1)
        # arm on launch if not waiting for service
        if self.tx_mode == Mode.PPM or self.tx_mode == Mode.SIM: 
            self.sequence_started = True
        self.get_logger().info('Initialization complete.') 

    @property
    def sequence_started(self):
        return self._sequence_started

    @sequence_started.setter
    def sequence_started(self, value):
        self._sequence_started = value
        if value:
            self.sequence_started_time = time.time()
            self.get_logger().info(f'Command squence started at {time.strftime("%Y-%m-%d %H:%M:%S GMT", time.gmtime(self.sequence_started_time))}')

    def call_telemetry_services(self):
        if self.battery_voltage_client.service_is_ready():
            battery_voltage_future = self.battery_voltage_client.call_async(GetBatteryVoltage.Request())
            battery_voltage_future.add_done_callback(self.battery_voltage_callback)
        if self.attitude_client.service_is_ready():
            att_future = self.attitude_client.call_async(GetAttitude.Request())
            att_future.add_done_callback(self.attitude_callback)
        elif self.pitch_client.service_is_ready() and self.roll_client.service_is_ready() and self.yaw_client.service_is_ready(): # use individual axis services
            pitch_future = self.pitch_client.call_async(GetPitch.Request())
            roll_future = self.roll_client.call_async(GetRoll.Request())
            yaw_future = self.yaw_client.call_async(GetYaw.Request())
            pitch_future.add_done_callback(self.pitch_callback)
            roll_future.add_done_callback(self.roll_callback)
            yaw_future.add_done_callback(self.yaw_callback)

    def pitch_callback(self, future):
        if future.result().success:
            self.pitch = future.result().radians
    def roll_callback(self, future):
        if future.result().success:
            self.roll = future.result().radians
    def yaw_callback(self, future):
        if future.result().success:
            self.yaw = future.result().radians

    def yaw_callback(self, future):
        if future.result().success:
            self.yaw = future.result().radians

    def attitude_callback(self, future):
        if future.result().success:
            self.pitch = future.result().pitch_radians
            self.roll = future.result().roll_radians
            self.yaw = future.result().yaw_radians
            # start programmed sequence as soon as service indicates readiness
            if not self.sequence_started and self.last_camera_recv_t:
                self.sequence_started = True
    
    
    def battery_voltage_callback(self, future):
        if future.result().success:
            self.battery_voltage = future.result().voltage
    
    def arm(self) -> None:
        self.get_logger().info('Arming...')
        self.tx.arm()

    def disarm(self) -> None:
        self.tx.disarm()

    def enable_auto(self) -> None:
        self.yaw_pid.set_auto_mode(True)
        self.throttle_pid.set_auto_mode(True)
        self.roll_pid.set_auto_mode(True)
        self.pitch_pid.set_auto_mode(True)
        

    def set_setpoint(self, new_setpoint: Position) -> None:
        self.roll_pid.setpoint = new_setpoint.x 
        self.throttle_pid.setpoint = new_setpoint.y 
        self.pitch_pid.setpoint = new_setpoint.z 
        self.yaw_pid.setpoint = new_setpoint.theta

    def get_current_position(self) -> Position: 
        return Position(self.x.value, self.y.value, self.z.value, self.theta)

    def start_takeoff(self) -> None: 
        self._taking_off = True
    def end_takeoff(self) -> None: 
        self._taking_off = False
        self.enable_auto()
    def control(self):
        # default values(permit arming)
        pitch = 1500
        roll = 1500
        throttle = 988
        yaw = 1500
        mode = 988

        if self.sequence_started:
            self.sequence.update(self)

           # logging msg initialization 
            velocity_msg = Float64MultiArray()
            velocity_msg.data = []
            velocity_target_msg = Float64MultiArray()
            velocity_target_msg.data = []
            acc_msg = Float64MultiArray()
            acc_msg.data = []
            acc_target_msg = Float64MultiArray()
            acc_target_msg.data = []
            # control
            if self.throttle_pid.auto_mode and self.y.filtering:
                outputs = self.throttle_pid(self.y.value, self.y.first_derivative_value, self.y.second_derivative_value)
                velocity_msg.data.append(self.y.first_derivative_value)
                velocity_target_msg.data.append(outputs[0])
                acc_msg.data.append(self.y.second_derivative_value)
                acc_target_msg.data.append(outputs[1])
                throttle = channel_value((self.throttle_ff + outputs[-1]) / (math.cos(self.pitch) * math.cos(self.roll)))
            else:
                throttle = channel_value(self.throttle_ff)

            if self.pitch_pid.auto_mode and self.z.filtering:
                outputs = self.pitch_pid(self.z.value, self.z.first_derivative_value, self.z.second_derivative_value) 
                velocity_msg.data.append(self.z.first_derivative_value)
                velocity_target_msg.data.append(outputs[0])
                acc_msg.data.append(self.z.second_derivative_value)
                acc_target_msg.data.append(outputs[1])
                pitch = channel_value(self.pitch_ff - outputs[-1] / (throttle / CONTROL_THROTTLE))
            else:
                pitch = channel_value(self.pitch_ff)

            if self.roll_pid.auto_mode and self.x.filtering:
                outputs = self.roll_pid(self.x.value, self.x.first_derivative_value, self.x.second_derivative_value)
                velocity_msg.data.append(self.x.first_derivative_value)
                velocity_target_msg.data.append(outputs[0])
                acc_msg.data.append(self.x.second_derivative_value)
                acc_target_msg.data.append(outputs[1])
                roll = channel_value(self.roll_ff - outputs[-1] / (throttle / CONTROL_THROTTLE))
            else:
                roll = channel_value(self.roll_ff)

            # inverse yaw
            if self.yaw_pid.auto_mode:
                yaw = channel_value(self.yaw_ff - self.yaw_pid(self.theta))
            else:
                yaw = channel_value(self.yaw_ff)
            
            # for logging w/ rosbags
            effort_msg = Float64MultiArray()
            effort_msg.data = [pitch, roll, yaw, throttle]
            self.effort_publisher.publish(effort_msg) 
            self.velocity_publisher.publish(velocity_msg)
            self.velocity_target_publisher.publish(velocity_target_msg)
            self.acc_publisher.publish(acc_msg)
            self.acc_target_publisher.publish(acc_target_msg)
        # manual control enabled
        if self.manual_rc_channels:
            if self.config.manual['roll']:
                roll = self.manual_rc_channels[0]
            if self.config.manual['pitch']:
                pitch = self.manual_rc_channels[1]
            if self.config.manual['throttle']:
                throttle = self.manual_rc_channels[2]
            if self.config.manual['yaw']:
                yaw = self.manual_rc_channels[3]
            if self.config.manual['mode']:
                mode = self.manual_rc_channels[5]
        if self._taking_off:
            throttle = self.config.throttle_ff
        if self.pitch_pid.auto_mode or self.roll_pid.auto_mode or self.throttle_pid.auto_mode or self.yaw_pid.auto_mode or (self.manual_rc_channels and self.config.manual['throttle']):
            self.tx.update(pitch=pitch, roll=roll, yaw=yaw, throttle=throttle, mode=mode)

    def joystick_listener_callback(self, msg: Int32MultiArray):
        self.manual_rc_channels = msg.data

    def listener_callback(self, msg: Float64MultiArray):
        raw_x, raw_y, raw_z, raw_theta, t = msg.data
        self.theta = raw_theta
        # scale to meters
        raw_x /= 100
        raw_y /= 100
        raw_z /= 100
        if self.last_camera_recv_t:
            sample_time = (time.time() - self.last_camera_recv_t)
            self.get_logger().debug(f'RECV FPS: {1 / sample_time}')
            self.x.add(raw_x, sample_time)
            self.y.add(raw_y, sample_time)
            self.z.add(raw_z, sample_time)
            # low-pass filtering
            self.theta = -0.35367012 * self.prev_theta + 0.67683506 * raw_theta + 0.67683506 * self.prev_unfiltered_theta 
        self.get_logger().info(f'x: {self.x.value}, y: {self.y.value}, z: {self.z.value}, theta: {self.theta}')
        # preserving last state
        self.prev_theta = self.theta
        self.prev_unfiltered_theta = raw_theta
        self.last_camera_recv_t = t

    def exit_gracefully(self):
        self.get_logger().info('Exiting...')
        self.tx.exit_gracefully()
            
def main(args=None):
    # deal with --ros-args
    if '--ros-args' in sys.argv:
        idx = sys.argv.index('--ros-args')
        user_args = sys.argv[1:idx]
    else:
        user_args = sys.argv[1:]
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=Mode.from_string, choices=list(Mode), required=True,
                        help='Choose the tx type: sim for gazebo simulation, ppm for a microcontroller interface(e.g. arduino) or elrs for direct serial control of an elrs module')
    parsed_args = parser.parse_args(user_args) 
    signal.signal(signal.SIGTERM, sig_handler)
    signal.signal(signal.SIGINT, sig_handler)
    rclpy.init(args=args, signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    controller = None
    try:
        controller = PIDController(parsed_args.mode)
        while rclpy.ok():
            rclpy.spin_once(controller, timeout_sec=0.01)
    except (KeyboardInterrupt, SystemExit):
        if controller is not None:
            controller.get_logger().fatal('Emergency stop!')
            controller.exit_gracefully()
        else:
            print('Emergency stop!')
    finally:
        if controller is not None:
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()