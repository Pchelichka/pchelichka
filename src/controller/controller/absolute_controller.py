import rclpy
from rclpy.node import Node
import time
from .tx import Tx
from .pid import PID, CascadePID
from .savgol_filtered_data import SavgolFilteredData
from std_msgs.msg import Float64MultiArray
import math
from telemetry_interfaces.srv import GetPitch, GetRoll, GetYaw, GetBatteryVoltage

def clip(value: int, lower: int, upper: int):
    return lower if value < lower else upper if value > upper else value

class PIDController(Node):

    def __init__(self):
        super().__init__('controller')
        self.armed = False
        self.pollinated = False
        self.landing = False
        self.effort_publisher = self.create_publisher(Float64MultiArray, 'effort', 10) 
        self.velocity_publisher = self.create_publisher(Float64MultiArray, 'velocity', 10) 
        self.velocity_target_publisher = self.create_publisher(Float64MultiArray, 'velocity_target', 10) 
        self.acc_publisher = self.create_publisher(Float64MultiArray, 'acc', 10) 
        self.acc_target_publisher = self.create_publisher(Float64MultiArray, 'acc_target', 10) 
        # self.roll_pid = PID(1, 0, 0, setpoint=0) # 270, 40, 40
        # self.roll_vel_pid = PID(350, 0, 0, setpoint=0) # 270, 40, 40
        self.roll_pid = CascadePID(3, 0, 
                                   1, 0, 0,   # pos
                                   1.5, 1.2, 0, # vel 
                                   45, 3, 0)   # acc
        # self.pitch_pid = PID(1, 0, 0, setpoint=0.5) # 120, 4, 50 # 500, 40, 50
        # self.pitch_vel_pid = PID(350, 0, 0, setpoint=0) # 270, 40, 40
        self.pitch_pid = CascadePID(3, 0.75, 
                                   1, 0, 0,   # pos 
                                    1.5, 1.2, 0, # vel
                                    45, 3, 0)   # acc
        self.yaw_pid = PID(80, 0, 10, setpoint=0)
        # self.throttle_pid = PID(1, 0, 0, setpoint=0) #22, 1.2, 40; 30, 18, 11.25
        # self.throttle_vel_pid = PID(10, 0, 0, setpoint=0) #22, 1.2, 40; 30, 18, 11.25
        self.throttle_pid = CascadePID(3, 0, 
                                       1.5, 0.2, 0,  # pos
                                       4, 8, 0, # vel # 1.2 
                                       20, 10, 0)  # acc # 1.8
        self.throttle_ff = 1395
        self.roll_ff = 1500
        self.pitch_ff = 1500
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.battery_voltage = 0
        self.x = SavgolFilteredData(1 / 30)
        self.y = SavgolFilteredData(1 / 30)
        self.z = SavgolFilteredData(1 / 30)
        self.prev_theta = None
        self.prev_unfiltered_theta = None
        self.prev_t = None
        self.pitch_client = self.create_client(GetPitch, '/get_pitch')
        self.roll_client = self.create_client(GetRoll, '/get_roll')
        self.yaw_client = self.create_client(GetYaw, '/get_yaw')
        self.battery_voltage_client = self.create_client(GetBatteryVoltage, '/get_battery_voltage')

        while not self.pitch_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Pitch service not available, waiting again...')
        while not self.roll_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Roll service not available, waiting again...')
        while not self.yaw_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Yaw service not available, waiting again...')
        while not self.battery_voltage_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Battery voltage service not available, waiting again...')
        self.timer_action = self.create_timer(0.01, self.call_telemetry_services)
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
        self.armed_time = time.time()
    def call_telemetry_services(self):
        pitch_future = self.pitch_client.call_async(GetPitch.Request())
        roll_future = self.roll_client.call_async(GetRoll.Request())
        yaw_future = self.yaw_client.call_async(GetYaw.Request())
        battery_voltage_future = self.battery_voltage_client.call_async(GetBatteryVoltage.Request())
        pitch_future.add_done_callback(self.pitch_callback)
        roll_future.add_done_callback(self.roll_callback)
        yaw_future.add_done_callback(self.yaw_callback)
        battery_voltage_future.add_done_callback(self.battery_voltage_callback)
    def pitch_callback(self, future):
        if future.result().success:
            self.pitch = future.result().radians
    def roll_callback(self, future):
        if future.result().success:
            self.roll = future.result().radians
    def yaw_callback(self, future):
        if future.result().success:
            self.yaw = future.result().radians
    
    def battery_voltage_callback(self, future):
        if future.result().success:
            self.battery_voltage = future.result().voltage

    def listener_callback(self, msg: Float64MultiArray):
        raw_x, raw_y, raw_z, raw_theta, t = msg.data
        raw_x /= 100
        self.x.add(raw_x)
        # x = raw_x
        raw_y /= 100
        self.y.add(raw_y)
        # y = raw_y
        raw_z /= 100
        self.z.add(raw_z)
        # z = raw_z
        theta = raw_theta
        if self.prev_t:
            # low-pass filtering
            # x = -0.35367012 * self.prev_x + 0.67683506 * raw_x + 0.67683506 * self.prev_unfiltered_x
            # y = -0.35367012 * self.prev_y + 0.67683506 * raw_y + 0.67683506 * self.prev_unfiltered_y
            # z = -0.35367012 * self.prev_z + 0.67683506 * raw_z + 0.67683506 * self.prev_unfiltered_z
            theta = -0.35367012 * self.prev_theta + 0.67683506 * raw_theta + 0.67683506 * self.prev_unfiltered_theta
        if self.armed and self.prev_t:
            # self.yaw_pid.set_auto_mode(True)
            if time.time() - self.armed_time > 0.3 and not self.landing:
                if not self.throttle_pid.auto_mode:
                    self.throttle_pid.set_auto_mode(True)
                    self.throttle_ff = 1390
            elif self.landing:
                self.throttle_pid.set_auto_mode(False)
                self.throttle_ff = 1300
            else: 
                self.throttle_pid.set_auto_mode(False)
                self.throttle_ff = 1420
            # self.roll_pid.set_auto_mode(True)
            # self.roll_vel_pid.set_auto_mode(True)
            # self.pitch_pid.set_auto_mode(True)
            # self.pitch_vel_pid.set_auto_mode(True)
            # self.throttle_vel_pid.set_auto_mode(True)
            self.roll_ff = 1500 #1580
            self.pitch_ff = 1500 # 1530
            # else:
            #     self.roll_pid.set_auto_mode(False)
            #     self.roll_vel_pid.set_auto_mode(False)
            #     self.pitch_pid.set_auto_mode(False)
            #     self.pitch_vel_pid.set_auto_mode(False)
            #     self.throttle_pid.set_auto_mode(False)
            #     self.throttle_vel_pid.set_auto_mode(False)
            #     self.throttle_ff = 1400
            #     self.roll_ff = 1500
            #     self.pitch_ff = 1500
            velocity_msg = Float64MultiArray()
            velocity_msg.data = []
            velocity_target_msg = Float64MultiArray()
            velocity_target_msg.data = []
            acc_msg = Float64MultiArray()
            acc_msg.data = []
            acc_target_msg = Float64MultiArray()
            acc_target_msg.data = []
            if self.throttle_pid.auto_mode and self.y.filtering:
                outputs = self.throttle_pid(self.y.value, self.y.first_derivative_value, self.y.second_derivative_value)
                print(outputs)
                velocity_msg.data.append(self.y.first_derivative_value)
                velocity_target_msg.data.append(outputs[0])
                acc_msg.data.append(self.y.second_derivative_value)
                acc_target_msg.data.append(outputs[1])
                throttle = clip(round((self.throttle_ff + outputs[-1]) / (math.cos(self.pitch) * math.cos(self.roll))), 1000, 2000)#(math.cos(self.pitch) * math.cos(self.roll))
            else:
                throttle = self.throttle_ff

            if self.pitch_pid.auto_mode and self.z.filtering:
                outputs = self.pitch_pid(self.z.value, self.z.first_derivative_value, self.z.second_derivative_value)
                if self.pitch_pid.setpoint > 0.2 and abs(self.x.value) < 0.1 and abs(self.y.value) < 0.3 and abs(self.pitch_pid.setpoint - self.z.value) < 0.1:
                    self.pitch_pid.setpoint -= 0.0002 
                if self.z.value <= 0.25:
                    self.pollinated = True
                if self.pollinated:
                    self.pitch_pid.setpoint += 0.02
                    if self.z.value >= 0.5:
                        self.pollinated = False
                        self.landing = True
                velocity_msg.data.append(self.z.first_derivative_value)
                velocity_target_msg.data.append(outputs[0])
                acc_msg.data.append(self.z.second_derivative_value)
                acc_target_msg.data.append(outputs[1])
                pitch = clip(round((self.pitch_ff - outputs[-1])), 1000, 2000)
            else:
                pitch = self.pitch_ff
            if self.roll_pid.auto_mode and self.x.filtering:
                outputs = self.roll_pid(self.x.value, self.x.first_derivative_value, self.x.second_derivative_value)
                velocity_msg.data.append(self.x.first_derivative_value)
                velocity_target_msg.data.append(outputs[0])
                acc_msg.data.append(self.x.second_derivative_value)
                acc_target_msg.data.append(outputs[1])
                roll = clip(round((self.roll_ff - outputs[-1])), 1000, 2000)
            else:
                roll = self.roll_ff

            # inverse yaw
            if self.yaw_pid.auto_mode:
                yaw = clip(1500 - round(self.yaw_pid(theta)), 1000, 2000)
            else:
                yaw = 1500
            effort_msg = Float64MultiArray()
            effort_msg.data = [pitch, roll, yaw, throttle]
            self.effort_publisher.publish(effort_msg) 
            self.velocity_publisher.publish(velocity_msg)
            self.velocity_target_publisher.publish(velocity_target_msg)
            self.acc_publisher.publish(acc_msg)
            self.acc_target_publisher.publish(acc_target_msg)
            self.tx.update(pitch=pitch, roll=roll, yaw=yaw, throttle=throttle)
        self.prev_theta = theta
        self.prev_unfiltered_theta = raw_theta
        self.prev_t = t

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