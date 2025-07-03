import socket
import time
import struct
import serial
from enum import Enum, auto
from std_msgs.msg import Int32MultiArray

class Mode(Enum):
    SIM = auto()
    PPM = auto()
    ELRS = auto()

    def __str__(self):
        return self.name.lower()

    @classmethod
    def from_string(cls, s):
        try:
            return cls[s.upper()]
        except KeyError:
            raise ValueError(f"Invalid mode: {s}. Valid options: {', '.join(m.name.lower() for m in cls)}")

class Tx:
    def __init__(self, mode=Mode.SIM, publisher=None):
        self.mode = mode

        if self.mode == Mode.SIM:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.client_socket.settimeout(1.0)
            self.channels = [1000] * 17
            self.channels[3] = 1500
            self.addr = ("127.0.0.1", 9004)
            self.send()
        elif self.mode == Mode.PPM:
            self.dev = serial.Serial(port='/dev/ttyACM0', baudrate=115200)
            self.channels = [1000] * 7
            self.send()
        elif self.mode == Mode.ELRS:
            self.channels = [1500] * 17
            self.publisher = publisher
        else:
            raise ValueError(f"Unsupported mode: {mode}")

        time.sleep(1)

    def send(self):
        if self.mode == Mode.SIM:
            self.channels[0] = time.time()
            self.client_socket.sendto(struct.pack('d' + 'H'*(len(self.channels) - 1), *self.channels), self.addr)
        elif self.mode == Mode.PPM:
            chns = [0, self.channels[3], self.channels[1], self.channels[2], self.channels[4], self.channels[5], self.channels[6]]
            for ch in chns[1:]:
                self.dev.write(round((ch - 1000) / 4).to_bytes(1, 'little', signed=False))
            self.dev.write(b'\n')
            self.dev.flush()
        elif self.mode == Mode.ELRS:
            msg = Int32MultiArray()
            msg.data = self.channels[1:]
            self.publisher.publish(msg)
        else:
            raise ValueError(f"Unsupported mode: {self.mode}")


    def arm(self):
        self.channels[1] = 1500
        self.channels[2] = 1500
        self.channels[3] = 988
        self.channels[4] = 1500
        self.channels[5] = 1999
        self.channels[6] = 988
        self.send()

    def land(self):
        self.set_throttle(1300)
        time.sleep(0.8)
        self.disarm()

    def set_throttle(self, throttle: int):
        self.channels[3] = throttle
        self.send()

    def update(self, pitch: int = 1500, roll: int = 1500, yaw: int = 1500, throttle: int = 1500):
        self.channels[1] = roll
        self.channels[2] = pitch
        self.channels[3] = throttle
        self.channels[4] = yaw
        self.send()

    def disarm(self):
        self.channels[1] = 1500
        self.channels[2] = 1500
        self.channels[3] = 988
        self.channels[4] = 1500
        self.channels[5] = 988
        self.send()

    def exit_gracefully(self):
        if self.mode == Mode.SIM:
            self.client_socket.close()
        elif self.mode == Mode.PPM:
            self.disarm()
            self.dev.close()
        elif self.mode == Mode.ELRS:
            self.disarm()
            
