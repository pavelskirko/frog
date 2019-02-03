# This example code is in the Public Domain (or CC0 licensed, at your option.)

# Unless required by applicable law or agreed to in writing, this
# software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied.

# -*- coding: utf-8 -*-

from builtins import input
import socket
import sys
import mssg_pb2

# -----------  Config  ----------
PORT = 3000
IP_VERSION = 'IPv4'
IPV4 = '192.168.0.1'
# -------------------------------

family_addr = socket.AF_INET
host = IPV4
accel = mssg_pb2.Accel()
try:
    sock = socket.socket(family_addr, socket.SOCK_STREAM)
except socket.error as msg:
        print('Could not create socket: ' + str(msg[0]) + ': ' + msg[1])
        sys.exit(1)

try:
    sock.connect((host, PORT))
except socket.error as msg:
        print('Could not open socket: ', msg)
        sock.close()
        sys.exit(1)
data_ar = []
num = 100
while num > 0:
    # msg = input('Enter message to send: ')
    # assert isinstance(msg, str)
    # msg = msg.encode()
    # sock.sendall(msg)
    data = sock.recv(1024)
    print("received: ", len(data))
    if not data:
        continue
        print("there is no data")
    data_ar.append(data)
    num = num - 1

sock.close()
for el in data_ar:
    try:
        accel.ParseFromString(el)
        print(accel)
    except:
        print("Error")
print("Overall, ", len(data_ar), " elements")