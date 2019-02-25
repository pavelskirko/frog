from math import cos, sin, acos, sqrt, atan
import numpy as np
import mssg_pb2
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D 

class AccelData:
    def __init__(self, accel=np.array([0,0,0]), number=0, time=0):
        self.accel = np.array(accel)
        self.veloc = np.array([0, 0, 0])
        self.pos = np.array([0, 0, 0])
        self.number = number
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
        beta = -acos(-sqrt((y**2 + z**2) / (x**2 + y**2 + z**2)))
        return [alpha, beta, 0]


class LandGear:
    def __init__(self, up_acc_data, down_acc_data):
        self.up_acc_data = up_acc_data
        self.down_acc_data = down_acc_data
        self.i_down_acc_data = []

    def __str__(self):
        printed_string = 'UP                                   DOWN\n'
        for i in range(min([len(self.up_acc_data), len(self.down_acc_data)])):
            printed_string += str(self.up_acc_data[i].number) + "\ta= " + np.array_str(self.up_acc_data[i].accel) + "\tt= " + str(self.up_acc_data[i].time) \
            + "\t" + str(self.i_down_acc_data[i].number) + "\ta= " + np.array_str(self.i_down_acc_data[i].accel) + "\tt= " + str(self.i_down_acc_data[i].time) + "\n"
        return printed_string

    def process(self):
        self.fill_time_gaps()
        self.interpolate()

    def interpolate(self):
        for i, el in enumerate(self.up_acc_data):
            a = AccelData()
            if i > len(self.down_acc_data)-1:
                break
            if el.time < self.down_acc_data[i].time:
                num = i
                while el.time < self.down_acc_data[num].time:
                    if num > 0: 
                        num -= 1
                    else:
                        break
                for j in range(3):
                    a.accel[j] = np.interp(el.time, np.array([self.down_acc_data[num].time, self.down_acc_data[num+1].time]),\
                 np.array([self.down_acc_data[num].accel[j], self.down_acc_data[num+1].accel[j]]))
            if el.time > self.down_acc_data[i].time:
                num = i
                while el.time > self.down_acc_data[num].time:
                    if num < len(self.down_acc_data) - 1: 
                        num += 1
                    else:
                        break
                for j in range(3):
                    a.accel[j] = np.interp(el.time, np.array([self.down_acc_data[num-1].time, self.down_acc_data[num].time]),\
                 np.array([self.down_acc_data[num-1].accel[j], self.down_acc_data[num].accel[j]]))
            a.number = el.number
            np.append(self.i_down_acc_data, a)
                




    # def group_by_time(self):

    def fill_time_gaps(self):
        number = 0
        for i, el in enumerate(self.up_acc_data):
            if el.time > 0:
                if el.number == 0:
                    number = el.number
                    time = el.time
                else:
                    avrg_time = (el.time - time) / (el.number - number) # avrg time of measurements between nearest measured time
                    for j in range(1, el.number - number):
                        if self.up_acc_data[i - j].time == 0:
                            self.up_acc_data[i - j].time = el.time - avrg_time * (el.number - self.up_acc_data[i - j].number)
                    number = el.number
                    time = el.time
        number = 0
        for i, el in enumerate(self.down_acc_data):
            if el.time > 0:
                if el.number == 0:
                    number = el.number
                    time = el.time
                else:
                    if el.number == number: # temporary
                        continue
                    avrg_time = (el.time - time) / (el.number - number) # avrg time of measurements between nearest measured time
                    for j in range(1, el.number - number):
                         if self.down_acc_data[i - j].time == 0:
                            self.down_acc_data[i - j].time = el.time - avrg_time * (el.number - self.down_acc_data[i - j].number)
                    number = el.number
                    time = el.time


    def get_start_time(self):
        acc_mov_avrg = []
        avrg_const = 10
        acc_treshold = 500
        num = 0
        acc_section = 0
        for el in self.up_acc_data:
            acc_section += np.sqrt(np.sum(np.power(el.accel, 2)))
            num += 1
            if num == avrg_const:
                num = 0
                acc_mov_avrg.append(acc_section / avrg_const)
                acc_section = 0
                for i, avrg_el in enumerate(acc_mov_avrg):
                    if avrg_el - acc_mov_avrg[i] > acc_treshold:
                        return el.time

    def get_avrg_gravity_up(self, time):
        num = 0
        acc_gravity_up = 0
        for el in self.up_acc_data:
            if el.time < time:
                acc_gravity_up += np.sqrt(np.sum(np.power(el.accel, 2)))
                num += 1
            else:
                return acc_gravity_up / num

    def get_avrg_gravity_down(self, time):
        num = 0
        acc_gravity_down = 0
        for el in self.down_acc_data:
            if el.time < time:
                acc_gravity_down += np.sqrt(np.sum(np.power(el.accel, 2)))
                num += 1
            else:
                return acc_gravity_down / num

def deg(rad):
    return rad * 180 / 3.14

file = open("result1.txt", "rb")
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
land_gear = LandGear(up_acc, down_acc)
# land_gear.process()
# print(land_gear)
# land_gear.get_start_time()

# plt.plot([el.time for el in land_gear.up_acc_data], [np.sqrt(np.sum(np.power(el.accel, 2))) for el in land_gear.up_acc_data])


plt_list_up = [sqrt(el.a_x**2 + el.a_y**2 + el.a_z**2) for el in res_decoded.up][300:1000]
plt_list_down = [sqrt(el.a_x**2 + el.a_y**2 + el.a_z**2) for el in res_decoded.down][300:1000]
mov_avrg = []
sm = 0
avrg_const = 5
for i, el in enumerate(plt_list_up):
    if i > avrg_const and len(plt_list_up) - i > avrg_const:
        for j in range(int(i - avrg_const), int(i + avrg_const)):
            sm += plt_list_up[j]
        mov_avrg.append(sm / (2 * avrg_const))
        # print(mov_avrg)
        sm = 0
plt.subplot(2, 1, 1)
plt.plot(range(len(mov_avrg)),mov_avrg)
plt.subplot(2, 1, 2)

mov_avrg = []
sm = 0
for i, el in enumerate(plt_list_down):
    if i > avrg_const and len(plt_list_down) - i > avrg_const:
        for j in range(int(i - avrg_const), int(i + avrg_const)):
            sm += plt_list_down[j]
        mov_avrg.append(sm / (2 * avrg_const))
        # print(mov_avrg)
        sm = 0
plt.plot(range(len(mov_avrg)),mov_avrg)
# plt.plot(range(500), plt_list_up[500:1000], color="r")
# plt.plot(range(500), plt_list_down[500:1000], color="b")
plt.show()




