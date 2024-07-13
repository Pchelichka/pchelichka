import time
import socket
import struct

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket.settimeout(1.0)
channels =  [1000] * 17
addr = ("127.0.0.1", 9004)

channels[0] = time.time()
channels[3] = 1500
client_socket.sendto(struct.pack('d' + 'H'*(len(channels) - 1), *channels), addr)
time.sleep(1)
channels[0] = time.time()
channels[1] = 1500
channels[2] = 1500
channels[3] = 1500
channels[4] = 1500
channels[5] = 1700
client_socket.sendto(struct.pack('d' + 'H'*(len(channels) - 1), *channels), addr)