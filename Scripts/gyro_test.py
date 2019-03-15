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

b_success = b'\x18' # bytes(int("11111111", 2))
b_fail = b'\xe7' # 11100111
n_fail = 0
n_sucss = 0
res = mssg_pb2.FinalResult()

while True:

    data = sock.recv(1024)
    if not data:
        continue
        print("there is no data")
    try:
        accel.ParseFromString(data)
    except Exception as error:
        print('Caught this error: ' + repr(error))
        print("data = ", len(data))
        print(data)
        sock.sendall(b_fail)
        print(n_fail, n_sucss)
        continue
    print(accel, (accel.a_x**2 + accel.a_y**2 + accel.a_z**2)**(1/2),  (accel.g_x**2 + accel.g_y**2 + accel.g_z**2)**(1/2))
sock.close()
