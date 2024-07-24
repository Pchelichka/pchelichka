import socket
import time
import struct

class Tx:
    def __init__(self):
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_socket.settimeout(1.0)
        self.channels =  [1000] * 17
        self.channels[3] = 1500
        self.addr = ("127.0.0.1", 9004)
        self.send()
        time.sleep(1)
    def send(self):
        self.channels[0] = time.time()
        self.client_socket.sendto(struct.pack('d' + 'H'*(len(self.channels) - 1), *self.channels), self.addr)

    def arm(self):
        self.channels[1] = 1500
        self.channels[2] = 1500
        self.channels[3] = 1500
        self.channels[4] = 1500
        self.channels[5] = 1700
        self.send()
    def set_throttle(self, throttle: int):
        self.channels[3] = throttle
        self.send()
    def update(self, pitch: int = 1500, roll: int = 1500, yaw: int = 1500, throttle: int = 1500):
        self.channels[1] = roll
        self.channels[2] = pitch
        self.channels[3] = throttle
        self.channels[4] = yaw
        self.send()
