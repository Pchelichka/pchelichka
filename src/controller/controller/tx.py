import socket
import time
import struct
import serial

class Tx:
    def __init__(self, sim=True):
        self.sim = sim
        if sim:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.client_socket.settimeout(1.0)
            self.channels =  [1000] * 17
            self.channels[3] = 1500
            self.addr = ("127.0.0.1", 9004)
        else:
            self.dev = serial.Serial(port='/dev/ttyACM0', baudrate=115200)
            self.channels =  [1000] * 7

        self.send()
        time.sleep(1)
    
    def send(self):
        if self.sim:
            self.channels[0] = time.time()
            self.client_socket.sendto(struct.pack('d' + 'H'*(len(self.channels) - 1), *self.channels), self.addr)
        else:
            chns = [0, self.channels[3], self.channels[1], self.channels[2], self.channels[4], self.channels[5], self.channels[6]]
            for ch in chns[1:]:
                self.dev.write(round((ch - 1000) / 4).to_bytes(1, 'little', signed=False))
            self.dev.write(b'\n')
            self.dev.flush()

    def arm(self):
        self.channels[1] = 1500
        self.channels[2] = 1500
        self.channels[3] = 1000
        self.channels[4] = 1500
        self.channels[5] = 1999
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
        self.channels[3] = 1500
        self.channels[4] = 1500
        self.channels[5] = 1200
        self.send()
    def exit_gracefully(self):
        if self.sim:
            self.client_socket.close()
        else:
            self.dev.close()