import time
import socket
import struct

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket.settimeout(1.0)
channels =  [1600] * 17
addr = ("127.0.0.1", 9004)

channels[0] = time.time()
client_socket.sendto(struct.pack('d' + 'H'*(len(channels) - 1), *channels), addr)
