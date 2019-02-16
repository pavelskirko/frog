from math import cos, sin, acos, sqrt, atan
import numpy as np
import mssg_pb2

class AccelData:
    def __init__(self, accel, num, time):
        self.accel = np.array(accel)
        self.veloc = np.array([0, 0, 0])
        self.pos = np.array([0, 0, 0])
        self.num = num
        self.time = time

    def rotate(self, alpha, beta, gamma):
        rX = np.array([[1, 0, 0],
                        [0, cos(alpha), -sin(alpha)],
                        [0, sin(alpha), cos(alpha)]])
        rY =  np.array([[cos(beta), 0, sin(beta)],
                        [0, 1, 0],
                        [-sin(beta), 0, cos(beta)]])
        rZ =  np.array([[cos(gamma), -sin(gamma), 0],
                        [sin(gamma), cos(gamma), 0],
                        [0, 0, 1]])
        if alpha != 0:
            self.accel = rX @ self.accel
        if beta != 0:
            self.accel = rY @ self.accel
        if gamma != 0:
            self.accel = rZ @ self.accel

    def calculateGravityAngle(self):
        x, y, z = self.accel
        alpha = -acos(-z / sqrt(y**2 + z**2))
        beta = -acos(-sqrt((y**2 + z**2) / (x**2 + y**2 + z**2) ))
        return [alpha, beta, 0]


class LandGear:
    def __init__(self, up_acc_data, down_acc_data):
        self.up_acc_data = up_acc_data
        self.down_acc_data = down_acc_data

    def get_start_time(self):
        acc_mov_avrg = []
        avrg_const = 10
        acc_treshold = 500
        num = 0
        for el in self.up_acc_data:
            acc_section += sqrt(el.accel[0]**2 + el.accel[1]**2 + el.accel[2]**2)
            num++
            if num == avrg_const:
                num = 0
                acc_mov_avrg.append(acc_section / avrg_const)
                for avrg_el, i in enumerate(acc_mov_avrg):
                    if avrg_el - acc_mov_avrg > acc_treshold:
                        return el.time

    def get_avrg_gravity_up(self, time):
        num = 0
        for el in self.up_acc_data:
            if el.time < time:
                acc_gravity_up += sqrt(el.accel[0]**2 + el.accel[1]**2 + el.accel[2]**2)
                num++
            else:
                return acc_gravity_up / num

    def get_avrg_gravity_down(self, time):
        num = 0
        for el in self.down_acc_data:
            if el.time < time:
                acc_gravity_down += sqrt(el.accel[0]**2 + el.accel[1]**2 + el.accel[2]**2)
                num++
            else:
                return acc_gravity_down / num

def deg(rad):
    return rad * 180 / 3.14

file = open("result_tabletest.txt", "rb")
data = file.read()
res_decoded = mssg_pb2.FinalResult()
res_decoded.ParseFromString(data)
up_acc = []
down_acc = []
for el in res_decoded.up:
    acc = AccelData([el.a_x, el.a_y, el.a_z], el.number, el.time)
    up_acc.append(acc)
for el in res_decoded.down:
    acc = AccelData([el.a_x, el.a_y, el.a_z], el.number, el.time)
    down_acc.append(acc)




