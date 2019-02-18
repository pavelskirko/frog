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
data_ar1 = []
data_ar2 = []
b_success = b'\x18' # bytes(int("11111111", 2))
b_fail = b'\xe7' # 11100111
n_fail = 0
n_sucss = 0
res = mssg_pb2.FinalResult()

# a = mssg_pb2.Accel()
# a.last_msg = False;
# a.a_x = -16384;
# a.a_y = 6;
# a.a_z = 16384;
# a.time = 1;
# a.number = 1;
# a.up = True;

# data_encoded = a.SerializeToString()


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
    # if(data == data_encoded):
    #     print("the same")
    # else:
        # print("not the same")
        # print(data)
        # print(data_encoded)

    try:
        accel.ParseFromString(data)
    except Exception as error:
        print('Caught this error: ' + repr(error))
        print("data = ", len(data))
        print(data)
        sock.sendall(b_success)
        print(n_fail, n_sucss)
        n_fail += 1
        continue
    # print(accel)
    sock.sendall(b_success)
    n_sucss += 1
    print(n_fail, n_sucss)
    print(data)
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
