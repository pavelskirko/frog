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
respond = mssg_pb2.Accel()
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
data_ar1 = []
data_ar2 = []
res = mssg_pb2.FinalResult()
while True:
    # msg = input('Enter message to send: ')
    # assert isinstance(msg, str)
    # msg = msg.encode()
    # sock.sendall(msg)
    data = sock.recv(1024)
    # print("received: ", len(data))
    # if len(data) == 1:
    #     break
    if not data:
        continue
        print("there is no data")
    try:
        accel.ParseFromString(data)
    except:
        respond.success = False
        sock.sendall(respond.SerializeToString())
        # print("Error")
        continue
    respond.success = True
    sock.sendall(respond.SerializeToString())
    if accel.last_msg:
        # print(accel)
        break
    if accel.up:
        res.up.extend([accel])
    else:
        res.down.extend([accel])
print(res)
sock.close()
res_encoded = res.SerializeToString()
file = open("result.txt", "wb+")
file.write(res_encoded)
file.close()
# accelerations = []
# for el, i in enumerate(data_ar1):
    # single_acc = []
    # try:
    #     accel.ParseFromString(el)
    #     single_acc.append(accel)
    # except:
    #     print("Error")
    # try:
    #     accel.ParseFromString(data_ar2[i])
    #     single_acc.append(accel)
    # except:
    #     print("Error")
    # try:
    #     print("a1: ", single_acc[0], " a2: ", single_acc[1])
    # except:
    #     pass
    # accelerations.append(single_acc)
    # print(el)
# for el, i in enumerate(data_ar2):
#     print(el)
print("Overall, ", len(res.up), " + ", len(res.down), " elements")
