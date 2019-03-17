from math import cos, sin, acos, sqrt, atan, pi
import numpy as np
import mssg_pb2
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D 
from mpl_toolkits.mplot3d import Axes3D

class AccelData:
    def __init__(self, accel=[0,0,0], gyro=[0,0,0], number=0, time=0):
        self.accel = np.array(accel, dtype=np.float64)
        self.veloc = np.array([0, 0, 0], dtype=np.float64)
        self.pos = np.array([0, 0, 0], dtype=np.float64)
        self.gyro = np.array(gyro, dtype=np.float64)
        self.angle = np.array([0, 0, 0], dtype=np.float64)
        self.number = number
        self.time = time

    def rotate(self, angles):
        alpha, beta, gamma = angles
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
            self.gyro = rX @ self.gyro
            self.angle = rX @ self.angle
        if beta != 0:
            self.accel = rY @ self.accel
            self.gyro = rY @ self.gyro
            self.angle = rY @ self.angle
        if gamma != 0:
            self.accel = rZ @ self.accel
            self.gyro = rZ @ self.gyro
            self.angle = rZ @ self.angle

    def calculateGravityAngle(self):
        x, y, z = self.accel
        if x > 0 and y > 0 and z > 0:
            alpha = -acos(-z / sqrt(y**2 + z**2))
            beta = -acos(-sqrt((y**2 + z**2) / (x**2 + y**2 + z**2)))
        elif x < 0 and y > 0 and z > 0:
            alpha = -acos(-z / sqrt(y**2 + z**2))
            beta = -acos(sqrt((y**2 + z**2) / (x**2 + y**2 + z**2)))
        elif x > 0 and y < 0 and z > 0:
            alpha = acos(-z / sqrt(y**2 + z**2))
            beta = -acos(-sqrt((y**2 + z**2) / (x**2 + y**2 + z**2)))
        elif x > 0 and y > 0 and z < 0:
            alpha = acos(z / sqrt(y**2 + z**2))
            beta = acos(-sqrt((y**2 + z**2) / (x**2 + y**2 + z**2)))
        elif x > 0 and y < 0 and z < 0:
            alpha = -acos(z / sqrt(y**2 + z**2))
            beta = acos(-sqrt((y**2 + z**2) / (x**2 + y**2 + z**2)))
        elif x < 0 and y < 0 and z > 0:
            alpha = acos(-z / sqrt(y**2 + z**2))
            beta = acos(-sqrt((y**2 + z**2) / (x**2 + y**2 + z**2)))
        elif x < 0 and y > 0 and z < 0:
            alpha = acos(z / sqrt(y**2 + z**2))
            beta = -acos(-sqrt((y**2 + z**2) / (x**2 + y**2 + z**2)))
        elif x < 0 and y < 0 and z < 0:
            alpha = -acos(z / sqrt(y**2 + z**2))
            beta = -acos(-sqrt((y**2 + z**2) / (x**2 + y**2 + z**2)))
        return [alpha, beta, 0]


class LandGear:
    def __init__(self, up_acc_data, down_acc_data):
        self.up_acc_data = up_acc_data
        self.down_acc_data = down_acc_data
        self.i_down_acc_data = []
        self.point_of_start = [0, 0]
        self.point_of_finish = [0, 0]
        self.gravity = [0, 0]
        self.point_of_shock = [0, 0]
        self.stock_movement = []
        self.force = []

    def __str__(self):
        printed_string = 'UP                                      DOWN\n'
        for i in range(min([len(self.up_acc_data), len(self.i_down_acc_data)])):
            printed_string += str(self.up_acc_data[i].number) + "\ta= " + np.array_str(np.around(self.up_acc_data[i].accel, 1))  \
            + "\tg= " + np.array_str(np.around(self.up_acc_data[i].gyro, 1)) + "\ta= " + np.array_str(np.around(self.i_down_acc_data[i].accel, 1)) \
            + "\tg= " + np.array_str(np.around(self.i_down_acc_data[i].gyro, 1)) + "\tt= " + str(self.i_down_acc_data[i].time) + "\n" \
            # + "\t" + str(self.down_acc_data[i].number) + "\ta= " + np.array_str(self.down_acc_data[i].accel) + "\tt= " + str(self.down_acc_data[i].time) + "\n"
        return printed_string

    def process(self):
        self.fill_time_gaps()
        self.interpolate()
        self.cut_edges(32)
        self.match_numbers()
        self.point_of_start = self.get_start_time(0.02)
        self.point_of_finish = self.get_finish_time(0.018)
        self.gravity = [self.get_avrg_gravity_up(self.point_of_start[0]), self.get_avrg_gravity_down(self.point_of_start[0]), self.get_finish_gravity_up()]
        self.apply_gyro_offset()
        self.normalize_gyro(250)
        self.allign_to_start_gravity()
        # self.just_rotate_everything(np.array([45,0,45]))
        self.rotate_by_gyro_angle()
        
        # self.acc_rotation()
        self.add_gravity()
        self.point_of_shock = self.get_shock_time(10)
        
        self.zeroing_resting_state()
        # self.rotate_around_z()
        # self.moving_avrg_acc(1)
        self.calc_velocity()
        self.calc_position()
        self.shock_velocity()
        self.calculate_movement()
        self.calculate_force(100)
        self.plot_force_movement()
        # self.plot_3d_position()
        # self.plot_angle_time()
        # self.plot_acc_time()
        # self.plot_vel_time()
        # self.plot_ax_vel_time()

        # self.plot_ax_pos_time()
        # self.plot_pos_time()

    def shock_velocity(self):
        v_up = self.up_acc_data[self.point_of_shock[1]].veloc
        v_down = self.i_down_acc_data[self.point_of_shock[1]].veloc
        print(v_up, v_down)

    def get_finish_gravity_up(self):
        grav_arr = np.array([])
        for el in self.up_acc_data[self.point_of_finish[1]:]:
            grav_arr = np.append(grav_arr, np.sqrt(np.sum(np.power(el.accel, 2))))
        return np.mean(grav_arr)

    def plot_force_movement(self):
        fig1 = plt.figure(facecolor='white', figsize=(13,8))
        ax1 = plt.axes(frameon=False)
        ax1.get_xaxis().tick_bottom()
        ax1.get_yaxis().tick_left()
        plt.ylabel("", labelpad=15)
        plt.xlabel('$\\Delta x{, }mm$', horizontalalignment='right', x=1.0, fontsize='x-large')
        h = plt.ylabel('F, kN', verticalalignment='top', y=1.0, fontsize='x-large')
        h.set_rotation(0)
        max_point = np.argmax(self.stock_movement)
        plt.plot(self.stock_movement[:max_point], self.force[:max_point])
        plt.plot(self.stock_movement[max_point:], self.force[max_point:])
        xmin, xmax = ax1.get_xaxis().get_view_interval()
        ymin, ymax = ax1.get_yaxis().get_view_interval()
        ax1.add_artist(Line2D((xmin, xmax), (ymin, ymin), color='black', linewidth=2))
        ax1.add_artist(Line2D((xmin, xmin), (ymin, ymax), color='black', linewidth=2))
        plt.grid()
        plt.show()

    def acc_rotation(self):
        angle = self.up_acc_data[self.point_of_finish[1]].calculateGravityAngle()
        angle = np.array([rad_norm(ang) for ang in angle])
        angle_up_arr = np.array([angle])
        angle = self.i_down_acc_data[self.point_of_finish[1]].calculateGravityAngle()
        angle = np.array([rad_norm(ang) for ang in angle])
        angle_down_arr = np.array([angle])
        for el in self.up_acc_data[self.point_of_finish[1]+1:]:
            angle = el.calculateGravityAngle()
            angle = np.array([rad_norm(ang) for ang in angle])
            angle_up_arr = np.append(angle_up_arr, [angle], axis=0)
            angle = self.i_down_acc_data[el.number].calculateGravityAngle()
            angle = np.array([rad_norm(ang) for ang in angle])
            angle_down_arr = np.append(angle_down_arr, [angle], axis=0)
        angle_up = np.mean(angle_up_arr, axis=0)
        print(deg(angle_up))
        angle_up /= self.up_acc_data[self.point_of_shock[1]].time - self.up_acc_data[self.point_of_start[1]].time
        for el in self.up_acc_data[self.point_of_start[1]:self.point_of_shock[1]]:
            self.up_acc_data[el.number].rotate(angle_up * (el.time - self.up_acc_data[self.point_of_start[1]].time))
        for el in self.up_acc_data[self.point_of_shock[1]:]:
            self.up_acc_data[el.number].rotate(angle_up)
        # angle_down = np.mean(angle_down_arr, axis=0)
        # for el in angle_up_arr:
        #     print(el)
        # for el in angle_down_arr:
        #     print(el)

    def calculate_movement(self):
        mov_arr_up = np.array([])
        time_arr = np.array([0])
        for el in self.up_acc_data[self.point_of_shock[1]:self.point_of_finish[1]]:
            mov = np.abs(np.sqrt(np.sum(np.power(el.pos, 2))) - np.sqrt(np.sum(np.power(self.up_acc_data[el.number + 1].pos, 2))))
            mov_arr_up = np.append(mov_arr_up, [mov])
            time_arr = np.append(time_arr, [el.time])

        mov_arr_down = np.array([])
        for el in self.i_down_acc_data[self.point_of_shock[1]:self.point_of_finish[1]]:
            mov = np.abs(np.sqrt(np.sum(np.power(el.pos, 2))) - np.sqrt(np.sum(np.power(self.i_down_acc_data[el.number + 1].pos, 2))))
            mov_arr_down = np.append(mov_arr_down, [mov])

        mov_arr = np.array([0])
        counter = 10
        for i, el in enumerate(mov_arr_up):
            mov_arr = np.append(mov_arr, [mov_arr[-1] + mov_arr_up[i] - mov_arr_down[i]])
            if mov_arr[-1] < mov_arr[-2] and counter != 0:
                counter -= 1
            if counter == 0 and mov_arr[-1] > 0 and mov_arr[-2] < 0:
                break
        time_arr = time_arr[:len(mov_arr)]

        plt.plot(time_arr, mov_arr)
        plt.show()
        self.stock_movement = np.abs(mov_arr * 10**-12)

    def calculate_force(self, mass):
        acc_arr = np.array([0])
        time_arr = np.array([0])
        for el in self.up_acc_data[self.point_of_shock[1]:self.point_of_finish[1]]:
            acc_arr = np.append(acc_arr, np.abs(np.sqrt(np.sum(np.power(el.accel, 2))) - \
                np.sqrt(np.sum(np.power(self.i_down_acc_data[el.number].accel, 2)))))
            time_arr = np.append(time_arr, [el.time])
        # plt.plot(time_arr, acc_arr)
        # plt.show()
        acc_arr = acc_arr[:len(self.stock_movement)]
        self.force = acc_arr * mass * 10**-6    

    def get_shock_time(self, tolerance):
        for el in self.i_down_acc_data[self.point_of_start[1]:self.point_of_finish[1]]:
            if np.sqrt(np.sum(np.power(el.accel, 2))) > (tolerance+1) * self.gravity[0]:
                return el.time, el.number
        return 0, 0

    def rotate_around_z(self):
        acc_arr = np.array([self.up_acc_data[self.point_of_start[1]].accel])
        for el in self.up_acc_data[self.point_of_start[1]+1:self.point_of_shock[1]]:
            acc_arr = np.append(acc_arr, [el.accel], axis=0)
        accel = np.mean(acc_arr, axis=0)
        gamma = acos(accel[0] / sqrt(accel[0]**2 + accel[1]**2))
        for el in self.up_acc_data[self.point_of_start[1]:self.point_of_finish[1]]:
            self.up_acc_data[el.number].rotate([0,0,gamma])
        print(gamma)
        acc_arr = np.array([self.up_acc_data[self.point_of_start[1]].accel])
        for el in self.i_down_acc_data[self.point_of_start[1]+1:self.point_of_shock[1]]:
            acc_arr = np.append(acc_arr, [el.accel], axis=0)
        accel = np.mean(acc_arr, axis=0)
        gamma = acos(accel[0] / sqrt(accel[0]**2 + accel[1]**2))
        print(gamma)
        for el in self.i_down_acc_data[self.point_of_start[1]:self.point_of_finish[1]]:
            self.i_down_acc_data[el.number].rotate([0,0,gamma])

    def zeroing_resting_state(self):
        for el in self.up_acc_data[:self.point_of_start[1]]:
            self.up_acc_data[el.number].accel = np.array([0, 0, 0])
            self.up_acc_data[el.number].gyro = np.array([0, 0, 0])
            self.up_acc_data[el.number].veloc = np.array([0, 0, 0])
            self.i_down_acc_data[el.number].accel = np.array([0, 0, 0])
            self.i_down_acc_data[el.number].gyro = np.array([0, 0, 0])
        for el in self.up_acc_data[self.point_of_finish[1]:]:
            self.up_acc_data[el.number].accel = np.array([0, 0, 0])
            self.up_acc_data[el.number].gyro = np.array([0, 0, 0])
            self.i_down_acc_data[el.number].accel = np.array([0, 0, 0])
            self.i_down_acc_data[el.number].gyro = np.array([0, 0, 0])
            self.i_down_acc_data[el.number].veloc = np.array([0, 0, 0])



    def apply_gyro_offset(self):
        # TODO: use temperature instead
        gyro_arr = np.array([self.up_acc_data[0].gyro])
        for el in self.up_acc_data[1:self.point_of_start[1]]:
            if el.time < self.point_of_start[0]:
                gyro_arr = np.append(gyro_arr, [el.gyro], axis=0)
            else:
                break
        gyro_mean = np.mean(gyro_arr, axis=0)
        for i in range(len(self.up_acc_data)):
            self.up_acc_data[i].gyro -= gyro_mean

        gyro_arr = np.array([self.i_down_acc_data[0].gyro])
        for el in self.i_down_acc_data[1:self.point_of_start[1]]:
            if el.time < self.point_of_start[0]:
                gyro_arr = np.append(gyro_arr, [el.gyro], axis=0)
            else:
                break
        gyro_mean = np.mean(gyro_arr, axis=0)
        for i in range(len(self.i_down_acc_data)):
            self.i_down_acc_data[i].gyro -= gyro_mean

    def normalize_gyro(self, full_scale):
        scale_factor = full_scale / 32767 # dps precision
        for i in range(len(self.up_acc_data)):
            self.up_acc_data[i].gyro *=  scale_factor
        for i in range(len(self.i_down_acc_data)):
            self.i_down_acc_data[i].gyro *=  scale_factor

    def just_rotate_everything(self, angle):
        for i in range(len(self.up_acc_data)):
                self.up_acc_data[i].rotate(rad(angle))

    def rotate_by_gyro_angle(self):
        for i, el in enumerate(self.up_acc_data[self.point_of_start[1]-1:self.point_of_finish[1]]):
            d_t = self.up_acc_data[el.number+1].time - el.time
            rot = el.gyro
            d_angle = rot * d_t * 10**-6
            for j in range(el.number+1, self.point_of_finish[1]):
                self.up_acc_data[j].angle += d_angle
                self.up_acc_data[j].rotate(rad(-d_angle))
        print(self.up_acc_data[self.point_of_finish[1]-1].angle)

        for i, el in enumerate(self.i_down_acc_data[self.point_of_start[1]-1:self.point_of_finish[1]]):
            d_t = self.i_down_acc_data[el.number+1].time - el.time # microseconds
            rot = el.gyro # degrees per second
            d_angle = rot * d_t * 10**-6
            for j in range(el.number+1, self.point_of_finish[1]):
                self.i_down_acc_data[j].angle += d_angle
                self.i_down_acc_data[j].rotate(rad(-d_angle))

    def calc_velocity(self):
        for i, el in enumerate(self.up_acc_data[self.point_of_start[1]:self.point_of_shock[1]]):
            d_t = self.up_acc_data[el.number+1].time - el.time
            acc = (self.up_acc_data[el.number+1].accel + el.accel) / 2
            self.up_acc_data[el.number+1].veloc = el.veloc + acc * d_t
        for i, el in enumerate(self.i_down_acc_data[self.point_of_start[1]:self.point_of_shock[1]]):
            d_t = self.i_down_acc_data[el.number+1].time - el.time
            acc = (self.i_down_acc_data[el.number+1].accel + el.accel) / 2
            self.i_down_acc_data[el.number+1].veloc = el.veloc + acc * d_t
        v_up = self.up_acc_data[self.point_of_shock[1]].veloc
        v_down = self.i_down_acc_data[self.point_of_shock[1]].veloc
        v_up_abs = np.sqrt(np.sum(np.power(v_up, 2)))
        v_down_abs = np.sqrt(np.sum(np.power(v_down, 2)))
        self.up_acc_data[self.point_of_shock[1]].veloc = v_up * v_down_abs / v_up_abs
        self.i_down_acc_data[self.point_of_shock[1]].veloc = v_up * v_up_abs / v_down_abs
        for i, el in enumerate(self.up_acc_data[self.point_of_shock[1]:self.point_of_finish[1]]):
            d_t = self.up_acc_data[el.number+1].time - el.time
            acc = (self.up_acc_data[el.number+1].accel + el.accel) / 2
            self.up_acc_data[el.number+1].veloc = el.veloc + acc * d_t
        for i, el in enumerate(self.i_down_acc_data[self.point_of_shock[1]:self.point_of_finish[1]]):
            d_t = self.i_down_acc_data[el.number+1].time - el.time
            acc = (self.i_down_acc_data[el.number+1].accel + el.accel) / 2
            self.i_down_acc_data[el.number+1].veloc = el.veloc + acc * d_t

    def moving_avrg_acc(self, avrg_const):
        new_data = []
        for i, el in enumerate(self.up_acc_data[:-avrg_const]):
            if i >= avrg_const:
                avrg_arr = np.array([self.up_acc_data[i + avrg_const].accel], dtype=np.float64)
                a = AccelData()
                for j in range(-avrg_const, avrg_const):
                    avrg_arr = np.append(avrg_arr, np.array([self.i_down_acc_data[i + j].accel]), axis=0)
                a.accel = np.mean(avrg_arr, axis=0)
                a.gyro = el.gyro
                a.time = el.time
                a.number = el.number
                new_data.append(a)
        self.up_acc_data = new_data
        new_data = []
        for i, el in enumerate(self.i_down_acc_data[:-avrg_const]):
            if i >= avrg_const:
                avrg_arr = np.array([self.i_down_acc_data[i + avrg_const].accel], dtype=np.float64)
                a = AccelData()
                for j in range(-avrg_const, avrg_const):
                    avrg_arr = np.append(avrg_arr, np.array([self.i_down_acc_data[i + j].accel]), axis=0)
                a.accel = np.mean(avrg_arr, axis=0)
                a.gyro = el.gyro
                a.time = el.time
                a.number = el.number
                new_data.append(a)
        self.i_down_acc_data = new_data


    def calc_position(self):
        for i, el in enumerate(self.up_acc_data[:-1]):
            d_t = self.up_acc_data[i+1].time - el.time
            acc = (self.up_acc_data[i+1].accel + el.accel) / 2
            vel = (self.up_acc_data[i+1].veloc + el.veloc) / 2
            self.up_acc_data[i+1].pos = el.pos + el.veloc * d_t + (acc * d_t**2) / 2
        for i, el in enumerate(self.i_down_acc_data[:-1]):
            d_t = self.i_down_acc_data[i+1].time - el.time
            acc = (self.i_down_acc_data[i+1].accel + el.accel) / 2
            vel = (self.i_down_acc_data[i+1].veloc + el.veloc) / 2
            self.i_down_acc_data[i+1].pos = el.pos + el.veloc * d_t + (acc * d_t**2) / 2

    def allign_to_start_gravity(self):
        t, n = self.point_of_start
        alpha_arr = np.array([])
        beta_arr = np.array([]) 
        for i, el in enumerate(self.up_acc_data[:n]):
            angles = el.calculateGravityAngle()
            alpha_arr = np.append(alpha_arr, angles[0])
            beta_arr = np.append(beta_arr, angles[1])
        alpha = np.mean(alpha_arr)
        beta = np.mean(beta_arr)
        for i in range(len(self.up_acc_data)):
            self.up_acc_data[i].rotate([alpha, beta, 0])
        alpha_arr = np.array([])
        beta_arr = np.array([])
        for i, el in enumerate(self.i_down_acc_data[:n]):
            angles = el.calculateGravityAngle()
            alpha_arr = np.append(alpha_arr, angles[0])
            beta_arr = np.append(beta_arr, angles[1])
        alpha = np.mean(alpha_arr)
        beta = np.mean(beta_arr)
        for i in range(len(self.i_down_acc_data)):
            self.i_down_acc_data[i].rotate([alpha, beta, 0])

    def add_gravity(self):
        t, n = self.point_of_start
        g = 9.79341 * 1000
        g_avrg_up = self.gravity[0]
        g_avrg_down = self.gravity[1]
        g_avrg_up_finish = self.gravity[2]
        for el in self.up_acc_data[:self.point_of_start[1]]:
            self.up_acc_data[el.number].accel[2] += g_avrg_up
            self.up_acc_data[el.number].accel *= g / g_avrg_up
        for el in self.up_acc_data[self.point_of_start[1]:self.point_of_finish[1]]:
            g_avrg = (g_avrg_up_finish*(el.time - self.point_of_start[0])  + g_avrg_up * (self.point_of_finish[0] - el.time))\
                                 / (self.point_of_finish[0] - self.point_of_start[0])
            self.up_acc_data[el.number].accel[2] += g_avrg
            self.up_acc_data[el.number].accel *= g / g_avrg
        for el in self.up_acc_data[self.point_of_finish[1]:]:
            self.up_acc_data[el.number].accel[2] += g_avrg_up_finish
            self.up_acc_data[el.number].accel *= g / g_avrg_up_finish

        for i in range(len(self.i_down_acc_data)):
            # self.i_down_acc_data[i].accel = self.i_down_acc_data[i].accel * g / g_avrg_down
            self.i_down_acc_data[i].accel[2] += g_avrg_down
            self.i_down_acc_data[i].accel *= g / g_avrg_down

    def plot_angle_time(self):
        plt_list_up = [np.sqrt(np.sum(np.power(el.angle, 2))) for el in self.up_acc_data]
        plt_list_down = [np.sqrt(np.sum(np.power(el.angle, 2))) for el in self.i_down_acc_data]
        plt_list_time = [el.time for el in self.up_acc_data]
        plt.subplot(2, 1, 1)
        plt.plot(plt_list_time, plt_list_up)
        plt.plot(plt_list_time, plt_list_up)
        plt.subplot(2, 1, 2)
        plt.plot(plt_list_time, plt_list_down)
        plt.plot(plt_list_time, plt_list_down)
        plt.show()

    def plot_acc_time(self):
        plt_list_up = [np.sqrt(np.sum(np.power(el.accel, 2))) for el in self.up_acc_data]
        plt_list_down = [np.sqrt(np.sum(np.power(el.accel, 2))) for el in self.i_down_acc_data]
        plt_list_time = [el.time for el in self.up_acc_data]
        t1, num1 = self.point_of_start
        t2, num2 = self.point_of_shock
        t3, num3 = self.point_of_finish
        plt.subplot(2, 1, 1)
        plt.plot(plt_list_time[:num1], plt_list_up[:num1])
        plt.plot(plt_list_time[num1:num2], plt_list_up[num1:num2])
        plt.plot(plt_list_time[num2:num3], plt_list_up[num2:num3])
        plt.plot(plt_list_time[num3:], plt_list_up[num3:])
        plt.subplot(2, 1, 2)
        plt.plot(plt_list_time[:num1], plt_list_down[:num1])
        plt.plot(plt_list_time[num1:num2], plt_list_down[num1:num2])
        plt.plot(plt_list_time[num2:num3], plt_list_down[num2:num3])
        plt.plot(plt_list_time[num3:], plt_list_down[num3:])
        plt.show()

    def plot_vel_time(self):
        plt_list_up = [np.sqrt(np.sum(np.power(el.veloc, 2))) for el in self.up_acc_data]
        plt_list_down = [np.sqrt(np.sum(np.power(el.veloc, 2))) for el in self.i_down_acc_data]
        plt_list_time = [el.time for el in self.up_acc_data]
        t, num = self.point_of_start
        plt.subplot(2, 1, 1)
        plt.plot(plt_list_time[:num], plt_list_up[:num])
        plt.plot(plt_list_time[num:], plt_list_up[num:])
        plt.subplot(2, 1, 2)
        plt.plot(plt_list_time[:num], plt_list_down[:num])
        plt.plot(plt_list_time[num:], plt_list_down[num:])
        plt.show()

    def plot_ax_vel_time(self):
        plt_list_up_x = [el.veloc[0] for el in self.up_acc_data]
        plt_list_up_y = [el.veloc[1] for el in self.up_acc_data]
        plt_list_up_z = [el.veloc[2] for el in self.up_acc_data]
        plt_list_down_x = [el.veloc[0] for el in self.i_down_acc_data]
        plt_list_down_y = [el.veloc[1] for el in self.i_down_acc_data]
        plt_list_down_z = [el.veloc[2] for el in self.i_down_acc_data]
        plt_list_time = [el.time for el in self.up_acc_data]
        t, num = self.get_start_time()
        plt.subplot(3, 2, 1)
        plt.plot(plt_list_time, plt_list_up_x)
        plt.subplot(3, 2, 2)
        plt.plot(plt_list_time, plt_list_down_x)
        plt.subplot(3, 2, 3)
        plt.plot(plt_list_time, plt_list_up_y)
        plt.subplot(3, 2, 4)
        plt.plot(plt_list_time, plt_list_down_y)
        plt.subplot(3, 2, 5)
        plt.plot(plt_list_time, plt_list_up_z)
        plt.subplot(3, 2, 6)
        plt.plot(plt_list_time, plt_list_down_z)
        plt.show()

    def plot_ax_pos_time(self):
        plt_list_up_x = [el.pos[0] for el in self.up_acc_data]
        plt_list_up_y = [el.pos[1] for el in self.up_acc_data]
        plt_list_up_z = [el.pos[2] for el in self.up_acc_data]
        plt_list_down_x = [el.pos[0] for el in self.i_down_acc_data]
        plt_list_down_y = [el.pos[1] for el in self.i_down_acc_data]
        plt_list_down_z = [el.pos[2] for el in self.i_down_acc_data]
        plt_list_time = [el.time for el in self.up_acc_data]
        plt.subplot(3, 2, 1)
        plt.plot(plt_list_time, plt_list_up_x)
        plt.subplot(3, 2, 2)
        plt.plot(plt_list_time, plt_list_down_x)
        plt.subplot(3, 2, 3)
        plt.plot(plt_list_time, plt_list_up_y)
        plt.subplot(3, 2, 4)
        plt.plot(plt_list_time, plt_list_down_y)
        plt.subplot(3, 2, 5)
        plt.plot(plt_list_time, plt_list_up_z)
        plt.subplot(3, 2, 6)
        plt.plot(plt_list_time, plt_list_down_z)
        plt.show()

    def plot_3d_position(self):
        plt_list_up_x = [el.pos[0] for el in self.up_acc_data]
        plt_list_up_y = [el.pos[1] for el in self.up_acc_data]
        plt_list_up_z = [el.pos[2] for el in self.up_acc_data]
        plt_list_down_x = [el.pos[0] for el in self.i_down_acc_data]
        plt_list_down_y = [el.pos[1] for el in self.i_down_acc_data]
        plt_list_down_z = [el.pos[2] for el in self.i_down_acc_data]
        fig = plt.figure()
        ax = Axes3D(fig)
        ax.scatter(plt_list_up_x,plt_list_up_y,plt_list_up_z)
        ax.scatter(plt_list_down_x,plt_list_down_y,plt_list_down_z)
        plt.show()

    def plot_pos_time(self):
        plt_list_up = [np.sqrt(np.sum(np.power(el.pos, 2))) for el in self.up_acc_data]
        plt_list_down = [np.sqrt(np.sum(np.power(el.pos, 2))) for el in self.i_down_acc_data]
        plt_list_time = [el.time for el in self.up_acc_data]
        t, num = self.get_start_time()
        plt.subplot(2, 1, 1)
        plt.plot(plt_list_time[:num], plt_list_up[:num])
        plt.plot(plt_list_time[num:], plt_list_up[num:])
        plt.subplot(2, 1, 2)
        plt.plot(plt_list_time[:num], plt_list_down[:num])
        plt.plot(plt_list_time[num:], plt_list_down[num:])
        plt.show()


    def cut_edges(self, cut):
        self.up_acc_data = self.up_acc_data[cut:-cut]
        self.i_down_acc_data = self.i_down_acc_data[cut:-cut]
        while(len(self.up_acc_data) != len(self.i_down_acc_data)):
            if len(self.up_acc_data) < len(self.i_down_acc_data):
                self.i_down_acc_data = self.i_down_acc_data[1:]
            else:
                self.up_acc_data = self.up_acc_data[1:]

    def interpolate(self):
        for i, el in enumerate(self.up_acc_data):
            a = AccelData()
            if i > len(self.down_acc_data) - 1:
                break
            if el.time < self.down_acc_data[i].time:
                num = i
                while el.time < self.down_acc_data[num].time:
                    if num > 0: 
                        num -= 1
                    else:
                        break
                if num > 0:
                    for j in range(3):
                        a.accel[j] = np.interp(el.time, np.array([self.down_acc_data[num].time, self.down_acc_data[num+1].time]), \
                     np.array([self.down_acc_data[num].accel[j], self.down_acc_data[num+1].accel[j]]))
                        a.gyro[j] = np.interp(el.time, np.array([self.down_acc_data[num].time, self.down_acc_data[num+1].time]), \
                     np.array([self.down_acc_data[num].gyro[j], self.down_acc_data[num+1].gyro[j]]))  
                    a.number = el.number
                    a.time = el.time
            if el.time > self.down_acc_data[i].time:
                num = i
                while el.time > self.down_acc_data[num].time:
                    if num < len(self.down_acc_data) - 1: 
                        num += 1
                    else:
                        break
                if num < len(self.down_acc_data) - 1:
                    for j in range(3):
                        a.accel[j] = np.interp(el.time, np.array([self.down_acc_data[num-1].time, self.down_acc_data[num].time]),\
                     np.array([self.down_acc_data[num-1].accel[j], self.down_acc_data[num].accel[j]]))
                        a.gyro[j] = np.interp(el.time, np.array([self.down_acc_data[num-1].time, self.down_acc_data[num].time]),\
                     np.array([self.down_acc_data[num-1].gyro[j], self.down_acc_data[num].gyro[j]]))
                    a.number = el.number
                    a.time = el.time
            self.i_down_acc_data = np.append(self.i_down_acc_data, a)

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

    def match_numbers(self):
        for i, el in enumerate(self.up_acc_data):
            self.up_acc_data[i].number = i
        for i, el in enumerate(self.i_down_acc_data):
            self.i_down_acc_data[i].number = i

    def get_start_time(self, tolerancy):
        acc_avrg = np.array([np.sqrt(np.sum(np.power(self.up_acc_data[0].accel, 2)))])

        for i, el in enumerate(self.up_acc_data[1:]):
            acc_next = np.sqrt(np.sum(np.power(el.accel, 2)))
            if np.abs(acc_next - np.mean(acc_avrg)) > np.mean(acc_avrg) * tolerancy: 
                return el.time, i + 1
            acc_avrg = np.append(acc_avrg, np.sqrt(np.sum(np.power(el.accel, 2))))
        return self.up_acc_data[-1].time, len(self.up_acc_data)

    def get_finish_time(self, tolerancy):
        acc_avrg = np.array([np.sqrt(np.sum(np.power(self.up_acc_data[-1].accel, 2)))])

        for el in self.up_acc_data[:self.point_of_start[1]:-1]:
            acc_next = np.sqrt(np.sum(np.power(el.accel, 2)))
            if np.abs(acc_next - np.mean(acc_avrg)) > np.mean(acc_avrg) * tolerancy: 
                return el.time, el.number
            acc_avrg = np.append(acc_avrg, np.sqrt(np.sum(np.power(el.accel, 2))))
        return self.up_acc_data[-1].time, self.up_acc_data[-1].number

    def get_avrg_gravity_up(self, time):
        acc_gravity_up = np.array([])
        for el in self.up_acc_data:
            if el.time < time:
                acc_gravity_up = np.append(acc_gravity_up, np.sqrt(np.sum(np.power(el.accel, 2))))
            else:
                return np.mean(acc_gravity_up)

    def get_avrg_gravity_down(self, time):
        acc_gravity_down = np.array([])
        for el in self.down_acc_data:
            if el.time < time:
                acc_gravity_down = np.append(acc_gravity_down, np.sqrt(np.sum(np.power(el.accel, 2))))
            else:
                return np.mean(acc_gravity_down)

def deg(rad):
    return rad * 180 / 3.14 

def rad(deg):
    return deg * 3.14 / 180

def rad_norm(rad):
    while(rad < 0):
        rad += pi
    while(rad >  pi/2):
        rad -= pi
    return rad


# file = open("result3.txt", "rb")
file = open("result_with_gyro_6.txt", "rb")

data = file.read()
res_decoded = mssg_pb2.FinalResult()
res_decoded.ParseFromString(data)
# print(res_decoded)
up_acc = []
down_acc = []
for el in res_decoded.up:
    acc = AccelData([el.a_x, el.a_y, el.a_z], [el.g_x, el.g_y, el.g_z], el.number, el.time)
    up_acc.append(acc)
for el in res_decoded.down:
    acc = AccelData([el.a_x, el.a_y, el.a_z], [el.g_x, el.g_y, el.g_z], el.number, el.time)
    down_acc.append(acc)
land_gear = LandGear(up_acc, down_acc)

land_gear.process()
print(land_gear)




