from math import cos, sin, acos, sqrt, atan, pi
import numpy as np
import mssg_pb2
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D 
from mpl_toolkits.mplot3d import Axes3D
import scipy.optimize as opt
import string

def func_c(x, a, b, c, d):
    return a * np.power((x-550000), 4) + b * np.power((x-550000), 3) + c * np.power((x-550000), 2) + d

def der_func_c(x, a, b, c):
    return 4 * a * np.power((x-550000), 3) + 3 * b * np.power((x-550000), 2) + 2 * c * (x-550000)

def func_dc(x, a, b, c, d, e):
    m = 100
    g = 9.79341 * 1000
    return a * np.power((x - 1068174.8), 4) + b * np.power((x - 1068174.8), 3) + c * np.power(x - 1068174.8, 2)  + d

def der_func_dc(x, a, b, c, d):
    m = 100
    g = 9.79341 * 1000
    return 4 * a * np.power((x - 1068174.8), 3) + 3 * b * np.power(x - 1068174.8, 2) + 2 * c *(x - 1068174.8)

def approx_c(x, y, yerr):
    ap = opt.curve_fit(func_c, x, y, method="lm", sigma=yerr, absolute_sigma=True, maxfev=10000)
    perr = np.sqrt(np.diag(ap[1]))
    for i, el in enumerate(list(string.ascii_lowercase)[0:len(ap[0])]):
        print(str(el) + ': ' + str(ap[0][i]) + ' err: ' + str(perr[i]))
    return ap[0]

def approx_dc(x, y, yerr):
    ap = opt.curve_fit(func_dc, x, y, method="lm", sigma=yerr, absolute_sigma=True, maxfev=10000)
    perr = np.sqrt(np.diag(ap[1]))
    for i, el in enumerate(list(string.ascii_lowercase)[0:len(ap[0])]):
        print(str(el) + ': ' + str(ap[0][i]) + ' err: ' + str(perr[i]))
    return ap[0]


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
        self.c_compr = [0,0,0,0,0]
        self.c_decompr = [0,0,0,0,0]
        self.stock_movement_time_compr = [[0,0]]
        self.stock_movement_time_decompr = [[0,0]]
        self.full_decompr_time = 0

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
        # self.moving_avrg_acc(1)
        self.match_numbers()
        # self.point_of_start = self.get_start_time(0.02)
        # self.point_of_finish = self.get_finish_time(0.018)
        self.gravity = [self.get_avrg_gravity_up(self.point_of_start[0]), self.get_avrg_gravity_down(self.point_of_start[0]), self.get_finish_gravity_up()]
        # self.apply_gyro_offset()
        # self.normalize_gyro(250)
        # self.allign_to_start_gravity()
        # self.just_rotate_everything(np.array([45,0,45]))
        # self.rotate_by_gyro_angle()
        
        # self.acc_rotation()
        # self.add_gravity()
        # self.point_of_shock = self.get_shock_time(10)
        
        # self.zeroing_resting_state()
        # self.rotate_around_z()
        
        # self.calc_velocity()
        # self.calc_position()
        # self.shock_velocity()
        # self.calculate_movement(140)
        # self.calculate_force(100,140)
        # self.plot_force_movement()
        # self.veloc_approx()
        # self.plot_force_func_mov()
        # self.plot_3d_position()
        # self.plot_angle_time()
        # self.plot_acc_time()
        # self.plot_rel_ax_acc_time()
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
        list_to_delete = []
        for i, el in enumerate(self.force[:max_point]):
            if el < 0.3:
                list_to_delete.append(i)
        for i in sorted(list_to_delete, reverse=True):
            self.force = np.delete(self.force, i)
            self.stock_movement = np.delete(self.stock_movement, i)
        max_point = np.argmax(self.stock_movement)
        divider = 100
        max_mov = np.amax(self.stock_movement)
        avrg_force = [np.array([])]
        avrg_mov = [max_mov/divider]
        mov_slice = max_mov/divider
        for i, el in enumerate(self.stock_movement[:max_point]):
            if el < mov_slice:
                avrg_force[-1] = np.append(avrg_force[-1], self.force[i])
            else:
                mov_slice += max_mov/divider  
                avrg_force[-1] = np.mean(avrg_force[-1])
                if avrg_force[-1] != avrg_force[-1]:
                    avrg_force.pop()
                    avrg_mov.pop()
                avrg_force.append([np.array([])])
                    
                avrg_mov.append(mov_slice)
        avrg_force[-1] = np.mean(avrg_force[-1])
        t = np.arange(0, max_mov, 0.001)
        coeff = approx(avrg_mov, avrg_force)
        plt.plot(t, func(t, coeff[0], coeff[1], coeff[2], coeff[3], coeff[4]), color="#fcb716")
        plt.plot(avrg_mov, avrg_force,".", color="#fcb716", label="compression") # orange

        avrg_force = [np.array([])]
        avrg_mov = [max_mov]
        mov_slice = max_mov
        for i, el in enumerate(self.stock_movement[max_point:]):
            if el > mov_slice:
                avrg_force[-1] = np.append(avrg_force[-1], self.force[i])
            else:
                mov_slice -= max_mov/divider  
                avrg_force[-1] = np.mean(avrg_force[-1])
                if avrg_force[-1] != avrg_force[-1]:
                    avrg_force.pop()
                    avrg_mov.pop()
                avrg_force.append([np.array([])])        
                avrg_mov.append(mov_slice)
        avrg_force[-1] = np.mean(avrg_force[-1])
        t = np.arange(0, max_mov, 0.001)
        coeff = approx(avrg_mov, avrg_force)
        plt.plot(t, func(t, coeff[0], coeff[1], coeff[2], coeff[3], coeff[4]), color="#fcb716")
        plt.plot(avrg_mov, avrg_force, ".", color="#4daf4a", label="decompression") # green

        # plt.plot(self.stock_movement[:max_point], self.force[:max_point],".", color="#fcb716", label="compression") # orange
        # plt.plot(self.stock_movement[max_point:], self.force[max_point:], ".", color="#4daf4a", label="decompression") # green
        xmin, xmax = ax1.get_xaxis().get_view_interval()
        ymin, ymax = ax1.get_yaxis().get_view_interval()
        ax1.add_artist(Line2D((xmin, xmax), (ymin, ymin), color='black', linewidth=2))
        ax1.add_artist(Line2D((xmin, xmin), (ymin, ymax), color='black', linewidth=2))
        plt.grid()
        plt.legend()
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

    def calculate_movement(self, stiction):
        mov_arr_up = np.array([])
        time_arr = np.array([0])
        for el in self.up_acc_data[self.point_of_shock[1]+stiction:self.point_of_finish[1]]:
            mov = np.abs(np.sqrt(np.sum(np.power(el.pos, 2))) - np.sqrt(np.sum(np.power(self.up_acc_data[el.number + 1].pos, 2))))
            mov_arr_up = np.append(mov_arr_up, [mov])
            time_arr = np.append(time_arr, [el.time])

        mov_arr_down = np.array([])
        for el in self.i_down_acc_data[self.point_of_shock[1]+stiction:self.point_of_finish[1]]:
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
        # print(mov_arr)
        stiction = 0
        for i, el in enumerate(mov_arr[1:]):
            if mov_arr[i] > 0 and el < 0:
                stiction = i
                break
        # print(stiction)

        time_arr = time_arr[:len(mov_arr)]

        plt.plot(time_arr, np.abs(mov_arr * 10**-12))
        # plt.show()
        self.stock_movement = np.abs(mov_arr * 10**-12)


    def get_movement_time(self, n_compr, n_decompr):
        mov_arr_up = np.array([])
        time_arr = np.array([])
        for el in self.up_acc_data[self.point_of_shock[1]+n_compr[0]:self.point_of_shock[1]+n_compr[1]]:
            mov = np.abs(np.sqrt(np.sum(np.power(el.pos, 2))) - np.sqrt(np.sum(np.power(self.up_acc_data[el.number + 1].pos, 2))))
            mov_arr_up = np.append(mov_arr_up, [mov])
            time_arr = np.append(time_arr, [el.time])

        mov_arr_down = np.array([])
        for el in self.i_down_acc_data[self.point_of_shock[1]+n_compr[0]:self.point_of_shock[1]+n_compr[1]]:
            mov = np.abs(np.sqrt(np.sum(np.power(el.pos, 2))) - np.sqrt(np.sum(np.power(self.i_down_acc_data[el.number + 1].pos, 2))))
            mov_arr_down = np.append(mov_arr_down, [mov])

        mov_arr_compr = np.array([0])
        for i, el in enumerate(mov_arr_up):
            mov_arr_compr = np.append(mov_arr_compr, [mov_arr_compr[-1] + mov_arr_up[i] - mov_arr_down[i]])
        mov_arr_compr = np.abs(mov_arr_compr[1:])
        
        mov_arr_compr = np.append([mov_arr_compr], [time_arr], axis=0)


        mov_arr_up = np.array([])
        time_arr = np.array([])
        for el in self.up_acc_data[self.point_of_shock[1]+n_decompr:self.point_of_finish[1]]:
            if el.time < 945000:
                mov = np.abs(np.sqrt(np.sum(np.power(el.pos, 2))) - np.sqrt(np.sum(np.power(self.up_acc_data[el.number + 1].pos, 2))))
                mov_arr_up = np.append(mov_arr_up, [mov])
                time_arr = np.append(time_arr, [el.time])
            else:
                mov_arr_up = np.append(mov_arr_up, [mov_arr_up[-1]])
                time_arr = np.append(time_arr, [el.time])

        mov_arr_down = np.array([])
        for el in self.i_down_acc_data[self.point_of_shock[1]+n_decompr:self.point_of_finish[1]]:
            if el.time < 945000:
                mov = np.abs(np.sqrt(np.sum(np.power(el.pos, 2))) - np.sqrt(np.sum(np.power(self.i_down_acc_data[el.number + 1].pos, 2))))
                mov_arr_down = np.append(mov_arr_down, [mov])
            else:
                mov_arr_down = np.append(mov_arr_down, [mov_arr_down[-1]])

        mov_arr_decompr = np.array([mov_arr_compr[0][-1]])
        for i, el in enumerate(mov_arr_up):
            mov_arr_decompr = np.append(mov_arr_decompr, [mov_arr_decompr[-1] - np.abs(mov_arr_up[i] - mov_arr_down[i])])
            if mov_arr_decompr[-1] < 0:
                break
        mov_arr_decompr = np.abs(mov_arr_decompr[1:])
        mov_arr_compr[0] = mov_arr_compr[0] * 10**-12 # to milimeters
        mov_arr_decompr = mov_arr_decompr * 10**-12 # to milimeters
        time_arr = time_arr[:mov_arr_decompr.size]
        self.full_decompr_time = time_arr[-1]
        print(self.full_decompr_time)
        mov_arr_decompr = np.append([mov_arr_decompr], [time_arr], axis=0)
        # print(mov_arr_decompr)
        # print(mov_arr)
        # for i, el in enumerate(mov_arr[1:]):
        #     if mov_arr[i] > 0 and el < 0:
        #         stiction = i
        #         break
        # print(stiction)
        self.stock_movement_time_compr = mov_arr_compr
        self.stock_movement_time_decompr = mov_arr_decompr

    def plot_force_func_mov(self):
        plt.plot(self.stock_movement_time_compr[0], der_func(self.stock_movement_time_compr[1], self.c_compr[0],self.c_compr[1],self.c_compr[2]))
        plt.plot(self.stock_movement_time_decompr[0], der_func(self.stock_movement_time_decompr[1], self.c_decompr[0],self.c_decompr[1],self.c_decompr[2]))
        plt.show()

    def calculate_force(self, mass, stiction):
        acc_arr = np.array([0])
        time_arr = np.array([0])
        for el in self.up_acc_data[self.point_of_shock[1]+stiction:self.point_of_finish[1]]:
            acc_arr = np.append(acc_arr, np.abs(np.sqrt(np.sum(np.power(el.accel, 2))) - \
                np.sqrt(np.sum(np.power(self.i_down_acc_data[el.number].accel, 2)))))
            time_arr = np.append(time_arr, [el.time])
        # plt.plot(time_arr, acc_arr)
        # plt.show()
        acc_arr = acc_arr[:len(self.stock_movement)]
        self.force = acc_arr * mass * 10**-6   

    def veloc_approx(self):
        m = 100
        g = 9.79341 * 1000
        vel_arr = np.array([])
        time_arr = np.array([])
        for el in self.up_acc_data[self.point_of_shock[1]:self.point_of_finish[1]]:
            vel_arr = np.append(vel_arr, (np.sqrt(np.sum(np.power(el.veloc, 2))) \
                - np.sqrt(np.sum(np.power(self.i_down_acc_data[el.number].veloc, 2)))))
            time_arr = np.append(time_arr, el.time)
        for i, el in enumerate(vel_arr):
            if i>3 and vel_arr[i-2] > vel_arr[i-1] and vel_arr[i-1] > el and vel_arr[i+1] > el and vel_arr[i+2] > vel_arr[i+1]:
                n_start = i
                break
        # n_start = np.argmin(vel_arr)
        vel_arr = vel_arr * m # let it be impulse 

        vel_arr_compr = np.array([])
        time_arr_compr = np.array([])
        n_finish_compr = 0
        for i, el in enumerate(vel_arr[n_start:]):
            if el < 0:
                vel_arr_compr = np.append(vel_arr_compr, el)
                time_arr_compr = np.append(time_arr_compr, time_arr[i+n_start])
            else:
                n_finish_compr = i + n_start
                break
        n_start_dec = 0
        for i, el in enumerate(vel_arr[n_finish_compr:]):
            if vel_arr[n_finish_compr + i - 1] < 0 and el > 0:
                n_start_dec = n_finish_compr + i 
        vel_arr_decompr = np.array([])
        time_arr_decompr = np.array([])
        self.get_movement_time([n_start, n_finish_compr], n_start_dec)
        for i, el in enumerate(vel_arr[n_start_dec:]):
            if time_arr[i+n_start_dec] > 945000:
                # vel_arr_decompr = np.append(vel_arr_decompr, vel_arr_decompr[-1]) 
                # time_arr_decompr = np.append(time_arr_decompr, time_arr[i+n_start_dec])
                break
            vel_arr_decompr = np.append(vel_arr_decompr, el) 
            time_arr_decompr = np.append(time_arr_decompr, time_arr[i+n_start_dec])
            if time_arr_decompr[-1] > self.full_decompr_time:
                vel_arr = vel_arr[:i+n_start_dec]
                break
        time_arr = time_arr[:vel_arr.size]
        c_compr = approx_c(time_arr_compr, vel_arr_compr, [0.001] + [0.1]*(len(time_arr_compr)-1))
        c_decompr = approx_dc(time_arr_decompr, vel_arr_decompr,[0.0001] +[0.001]*10+ [0.1]*(len(time_arr_decompr)-141) + [0.01]*130)
        plt.subplot(1,2,1)
        plt.ylabel("", labelpad=15)
        plt.xlabel('$t{, }\\mu s$', horizontalalignment='right', x=1.0, fontsize='x-large')
        h = plt.ylabel('$V{, }\\frac{m}{s}$', verticalalignment='top', y=1.0, fontsize='x-large')
        h.set_rotation(0)
        plt.plot(time_arr, vel_arr, ".", ms=3)
        t = np.arange(time_arr_compr[0], time_arr_compr[-1], 1)
        plt.plot(t, func_c(t, c_compr[0], c_compr[1], c_compr[2], c_compr[3]), linewidth=2, color="#fcb716", label="compression")
        t = np.arange(time_arr_decompr[0],time_arr_decompr[-1], 1)
        plt.plot(t, func_dc(t, c_decompr[0], c_decompr[1], c_decompr[2], c_decompr[3], c_decompr[4]), linewidth=2, color="#4daf4a", label="decompression")
        plt.grid()
        plt.legend()
        plt.subplot(1,2,2)
        self.c_compr = c_compr
        self.c_decompr = c_decompr
        plt.ylabel("", labelpad=15)
        plt.xlabel('$\\Delta x{, }mm$', horizontalalignment='right', x=1.0, fontsize='x-large')
        h = plt.ylabel('F, kN', verticalalignment='top', y=1.0, fontsize='x-large')
        h.set_rotation(0)
        plt.plot(self.stock_movement_time_compr[0], (m * g + der_func_c(self.stock_movement_time_compr[1], self.c_compr[0],self.c_compr[1],self.c_compr[2])) * 10**-6,color="#fcb716", linewidth=2, label="compression") # to kN
        plt.plot([self.stock_movement_time_compr[0][-1], self.stock_movement_time_decompr[0][0]], \
            [(m * g + der_func_c(self.stock_movement_time_compr[1][-1], self.c_compr[0],self.c_compr[1],self.c_compr[2])) * 10**-6, \
            (m * g + der_func_dc(self.stock_movement_time_decompr[1][0], self.c_decompr[0],self.c_decompr[1],self.c_decompr[2], self.c_decompr[3])) * 10**-6], color="#4daf4a", linewidth=2)
        plt.plot(self.stock_movement_time_decompr[0], (m * g + der_func_dc(self.stock_movement_time_decompr[1], self.c_decompr[0],self.c_decompr[1],self.c_decompr[2], self.c_decompr[3])) * 10**-6, color="#4daf4a", linewidth=2,  label="decompression")
        print(self.stock_movement_time_decompr[1][0], func_c(self.stock_movement_time_compr[1][-1],self.c_compr[0],self.c_compr[1],self.c_compr[2],self.c_compr[3]))
        plt.grid()
        plt.legend()
        plt.show()


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
        # print(self.up_acc_data[self.point_of_finish[1]-1].angle)

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
        self.i_down_acc_data[self.point_of_shock[1]].veloc = v_down * v_up_abs / v_down_abs
        for i, el in enumerate(self.up_acc_data[self.point_of_shock[1]:self.point_of_finish[1]+1000]):
            d_t = self.up_acc_data[el.number+1].time - el.time
            acc = (self.up_acc_data[el.number+1].accel + el.accel) / 2
            self.up_acc_data[el.number+1].veloc = el.veloc + acc * d_t
        for i, el in enumerate(self.i_down_acc_data[self.point_of_shock[1]:self.point_of_finish[1]+1000]):
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
        # plt_list_up = [np.sqrt(np.sum(np.power(el.accel, 2))) for el in self.up_acc_data]
        # plt_list_down = [np.sqrt(np.sum(np.power(el.accel, 2))) for el in self.i_down_acc_data]
        plt_list_up = [el.accel[2] for el in self.up_acc_data]
        plt_list_down = [el.accel[2] for el in self.i_down_acc_data]
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

    def plot_rel_ax_acc_time(self):
        plt_list_xy = [np.sqrt((el.accel[0] + el.accel[1])**2) - np.sqrt((self.i_down_acc_data[el.number].accel[0]+self.i_down_acc_data[el.number].accel[1])**2) for el in self.up_acc_data]
        plt_list_z = [el.accel[2] - self.i_down_acc_data[el.number].accel[2] for el in self.up_acc_data]
        plt_list_time = [el.time for el in self.up_acc_data]
        plt.subplot(2,1,1)
        plt.plot(plt_list_time, plt_list_xy)
        plt.subplot(2,1,2)
        plt.plot(plt_list_time, plt_list_z)
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
file = open("test_0_num_0.txt", "rb")

data = file.read()
res_decoded = mssg_pb2.FinalResult()
res_decoded.ParseFromString(data)
print(res_decoded)
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
# print(land_gear)




