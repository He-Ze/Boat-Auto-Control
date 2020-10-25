from time import time, sleep  # 引入时间
import socket  # 套接字
from threading import Thread  # 多线程
import math  # 数学计算
import csv
import casadi as ca
import casadi.tools as ca_tools
from matplotlib import pyplot as plt
import numpy as np
import time


def write_data():
    global file_path, data_to_log
    with open(file_path, "a+") as log_file:
        writer = csv.writer(log_file)
        writer.writerow(data_to_log)


def get_angle(point_pre, point_post):
    temp_x = point_post[0] - point_pre[0]
    temp_z = point_post[1] - point_pre[1]
    temp_angle = math.atan2(temp_x, temp_z)
    if temp_angle >= 0:
        return temp_angle - math.pi
    else:
        return math.pi + temp_angle


def get_angle_dis(angle1, angle2):  # 逆时针方向为正，顺时针为负
    temp_dis = angle2 - angle1
    if temp_dis < -1 * math.pi:
        return temp_dis + 2 * math.pi
    elif temp_dis > math.pi:
        return temp_dis - 2 * math.pi
    else:
        return temp_dis


def point_pre_process():
    global target_pointy, target_pointx, points_angle
    points_angle.clear()
    i = 0
    while i < len(target_pointx) - 3:
        points_angle.append(
            get_angle([target_pointx[i], target_pointy[i]], [target_pointx[i + 2], target_pointy[i + 2]]))
        i = i + 1
    print(points_angle)


def get_dis(point_pre, point_post):
    return math.sqrt(math.pow(point_post[0] - point_pre[0], 2) + math.pow(point_post[1] - point_pre[1], 2))


def find_nearest_point():
    global target_num, boat, target_pointx, target_pointy, data_to_log
    i = 0
    temp_dis = 10000
    while i < len(target_pointx) - 11:
        dis = get_dis([target_pointx[i], target_pointy[i]], [boat[0], boat[1]])
        if temp_dis > dis:
            temp_dis = dis
            target_num = i + 3
        i = i + 1
    print("Nearset point is", target_num, temp_dis)
    # data_to_log[3] = target_num
    return


def find_deci_angle():
    global boat, target_num, points_angle, ki, decide_angle, data_to_log, max_miss
    delta1 = get_angle_dis(points_angle[target_num], boat[4])
    print("detal1 is ", delta1)
    dis_u2p = get_dis([boat[0], boat[1]], [target_pointx[target_num], target_pointy[target_num]])
    print("dis_u2p is ", dis_u2p)
    degree_usv = get_angle([boat[0], boat[1]], [target_pointx[target_num], target_pointy[target_num]])
    print("degree_usv is ", degree_usv)
    judge_degree = get_angle_dis(points_angle[target_num], degree_usv)
    print("judge_degree is ", judge_degree)
    if judge_degree < -math.pi / 2:
        temp_judge_degree = judge_degree + math.pi
    elif judge_degree > math.pi / 2:
        temp_judge_degree = -math.pi + judge_degree
    else:
        temp_judge_degree = -judge_degree
    e_dis = dis_u2p * math.sin(temp_judge_degree)
    print("e_dis is ", e_dis)
    if math.fabs(e_dis) > max_miss:
        max_miss = math.fabs(e_dis)
    # et = ki * e_dis / (math.sqrt(math.pow(boat[2], 2) + math.pow(boat[3], 2)) + 0.1)
    et = ki * e_dis / 3
    print("et is ", et)
    decide_angle = et + delta1
    '''
    data_to_log[4] = delta1
    data_to_log[5] = et
    data_to_log[6] = decide_angle
    data_to_log[7] = e_dis
    data_to_log[8] = max_miss
    data_to_log[12] = boat[4]
    '''
    print("decide_angle is ", decide_angle)
    return


def change_motor():
    global s, decide_angle, had_arrive_the_dis, data_to_log, if_arrived_current
    power_max = 180
    speed_max = 4.82
    if not had_arrive_the_dis:
        speed = 3
    else:
        speed = 0
    if decide_angle < 0:
        w = math.sqrt(-decide_angle / math.pi) * 1.3
    else:
        w = math.sqrt(decide_angle / math.pi) * 1.3
    N = (speed / speed_max) * power_max
    temp_N = power_max - N
    print("w is ", w)
    if decide_angle > 0:
        left = N + temp_N * w
        right = N - temp_N * w
    else:
        left = N - temp_N * w
        right = N + temp_N * w
    if math.fabs(left) > power_max:
        left = left / math.fabs(left) * power_max
    if math.fabs(right) > power_max:
        right = right / math.fabs(right) * power_max
    set_motor = 'setspeed'
    set_motor = set_motor + ' '
    set_motor = set_motor + str(left)
    set_motor = set_motor + ' '
    set_motor = set_motor + str(right)
    set_motor = set_motor + '\r'
    s.send(set_motor.encode("utf-8"))
    print("Motor is ", set_motor)
    data4 = s.recv(1024).decode("utf-8")
    '''
    data_to_log[1] = power_max
    data_to_log[2] = speed_max
    data_to_log[9] = speed
    data_to_log[10] =left
    data_to_log[11] = right
    '''
    print(data4)

# 由于迭代次数的限制，规划的路径可能会出现大幅度改变，从而导致USV运动发生抖动，导致斯坦李控制失常
# 为了避免这种情况，我们加入了滤波检测将变化巨大的路径删除重新规划，误差为规划路径点和上一次规划路径点的距离平方之和，阈值为10000（手动调出的）
def if_change_ref(x1,y1,x2,y2):
    error = 0
    thr = 50000
    for i in range(len(x1)):
        error = error + (x1[i]-x2[i])**2 + (y1[i]-y2[i])**2
    if error < thr:
        return True
    else:
        return True

def send_and_recv():  # 接收和发送
    global boat, had_arrive_the_dis, destination, target_pointx, target_pointy, des, target_num, goal_num, if_arrived_current
    print('start to ask the information\n')
    f = open(r'mpc.csv', 'w', encoding='utf-8')
    csv_writer = csv.writer(f)
    csv_writer.writerow(["x", "y"])
    flag = 0
    data = 'position'
    data = data + '\r'
    s.send(data.encode("utf-8"))
    data1 = s.recv(1024).decode("utf-8")  # 得到了消息就进行相关解析
    data1 = data1[1:len(data1) - 1]
    strlist = data1.split(',')
    boat[0] = float(strlist[0])  # x
    boat[1] = float(strlist[2])  # z
    data = 'rotation'
    data = data + '\r'
    s.send(data.encode("utf-8"))
    data3 = s.recv(1024).decode("utf-8")
    data3 = data3[1:len(data3) - 1]
    strlist3 = data3.split(',')
    boat[4] = 2 * math.asin(float((strlist3[1])))
    temp = boat[4]
    if temp < 0:
        temp = math.pi + temp
    else:
        temp = -math.pi + temp
    target_pointx, target_pointy = planning([boat[0], boat[1], temp], des[goal_num], 40)

    while not had_arrive_the_dis:
        sleep_time = 0.2
        # 询问位置
        data = 'position'
        data = data + '\r'
        s.send(data.encode("utf-8"))
        data1 = s.recv(1024).decode("utf-8")  # 得到了消息就进行相关解析
        data1 = data1[1:len(data1) - 1]
        strlist = data1.split(',')
        boat[0] = float(strlist[0])  # x
        boat[1] = float(strlist[2])  # z
        csv_writer.writerow([boat[0], boat[1]])

        # print('位置是:')
        # print(data1)
        # print('\n')

        # 询问速度（x,y,z）
        data = 'speed'
        data = data + '\r'
        s.send(data.encode("utf-8"))
        data2 = s.recv(1024).decode("utf-8")
        data2 = data2[1:len(data2) - 1]
        strlist2 = data2.split(',')
        boat[2] = float(str(strlist2[0]))  # speedx
        boat[3] = float(str(strlist2[2]))  # speedz
        # print('速度是:')
        # print(data2)
        # print('\n')
        data = 'rotation'
        data = data + '\r'
        s.send(data.encode("utf-8"))
        data3 = s.recv(1024).decode("utf-8")
        data3 = data3[1:len(data3) - 1]
        strlist3 = data3.split(',')
        boat[4] = 2 * math.asin(float((strlist3[1])))
        temp = boat[4]
        if temp < 0:
            temp = math.pi + temp
        else:
            temp = -math.pi + temp

        # print('船只朝向:')
        # print(boat[4])
        # print('\n')

        # print('______________________________________')
        # print(get_dis([boat[0], boat[1]], des[goal_num][:2]))
        # print(if_arrived_current)
        # print('______________________________________')
        if flag % 2 == 0:
            temp_target_pointx, temp_target_pointy = planning([boat[0], boat[1], temp], des[goal_num], 20)
            if if_change_ref(temp_target_pointx,temp_target_pointy,target_pointx,target_pointy) :
                target_pointx = temp_target_pointx
                target_pointy = temp_target_pointy
        # if get_dis([boat[0],boat[1]],des[goal_num][:2]) < 20:
        # if_arrived_current = True
        # else:
        # if_arrived_current = False
        if get_dis([boat[0], boat[1]], des[goal_num][:2]) < 5 and len(des) > 1:
            goal_num += 1
            target_pointx, target_pointy = planning([boat[0], boat[1], temp], des[goal_num], 20)
        if get_dis([boat[0], boat[1]], destination) < 5:
                    had_arrive_the_dis = True

        point_pre_process()
        find_nearest_point()
        find_deci_angle()
        change_motor()
        write_data()
        sleep(0.1)
        flag = flag + 1
    f.close()
    return


def plot_ponit():
    global target_pointx, target_pointy, boat, target_num, goal_num, des, obs
    n = 0
    Ex2 = 0
    x = []
    y = []
    plt.ion()
    plt.axis('equal')
    desx = []
    desy = []
    for i in range(len(des)):
        desx.append(des[i][0])
        desy.append(des[i][1])
    '''
    obsx = []
    obsy = []
    for i in range(len(obs)):
        obsx.append(obs[i][0])
        obsy.append(obs[i][1])
        '''
    # plt.grid(axis="y")
    # plt.plot([20, 20, 150, 280, 280], [20, 280, 150, 280, 20], color='g')
    # plt.plot([20, 20], [70,130], "bo")
    while True:
        plt.clf()
        plt.axis('equal')
        # plt.xlim(boat[0] - 20, boat[0] + 100)
        # plt.ylim(boat[1] - 20, boat[1] + 100)
        plt.xlabel(boat[0])
        plt.ylabel(boat[1])
        plt.title(goal_num)
        plt.plot(desx, desy, color='g')
        plt.plot(obs[0], obs[1], "bo")
        plt.plot(boat[0], [boat[1]], '*')
        plt.plot(target_pointx[target_num], target_pointy[target_num], 'o')
        plt.plot(target_pointx[target_num + 2], target_pointy[target_num + 2], '^')
        plt.plot(target_pointx, target_pointy, '.')

        # if goal_num == 0:
        #     horizon_error = ((des[goal_num + 1][1] - des[goal_num][1]) * boat[0] - (
        #             des[goal_num + 1][0] - des[goal_num][0]) * boat[1] + (
        #                              des[goal_num + 1][0] * des[goal_num][1] - des[goal_num + 1][1] * des[goal_num][
        #                          0])) / math.sqrt((des[goal_num + 1][0] - des[goal_num][0]) ** 2 + (
        #             des[goal_num + 1][1] - des[goal_num][1]) ** 2)
        # else:
        #     horizon_error = ((des[goal_num][1] - des[goal_num - 1][1]) * boat[0] - (
        #             des[goal_num][0] - des[goal_num - 1][0]) * boat[1] + (
        #                              des[goal_num][0] * des[goal_num - 1][1] - des[goal_num][1] * des[goal_num - 1][
        #                          0]))/ math.sqrt((des[goal_num][0] - des[goal_num - 1][0]) ** 2 + (
        #             des[goal_num][1] - des[goal_num - 1][1]) ** 2)
        # # if horizon_error > 5:
        # #     horizon_error = 5
        # # if horizon_error < -5:
        # #     horizon_error = -5
        # Ex2 = (Ex2 * n + horizon_error*horizon_error)/(n+1)
        # n = n + 1
        # # plt.plot(n,math.sqrt(Ex2), 'ro')
        # # plt.plot(n, horizon_error, 'r.')
        # # plt.plot(n, math.sqrt((obs[0][0] - boat[0]) * (obs[0][0] - boat[0]) + (obs[1][0] - boat[1]) * (obs[1][0] - boat[1])), 'b.')
        # # plt.plot(n, math.sqrt((obs[0][1] - boat[0]) * (obs[0][1] - boat[0]) + (obs[1][1] - boat[1]) * (obs[1][1] - boat[1])), 'b.')
        # # plt.plot(boat[0], boat[1], 'r.')
        x.append(boat[0])
        y.append(boat[1])
        plt.plot(x, y, 'b.')

        plt.pause(0.01)
        plt.ioff()


def planning(x_c, x_s, N_pre):
    global data_to_log, goal_num, des, if_arrived_current, had_arrive_the_dis
    # 取样时间，用于估计未来状态的
    T = 0.3  # sampling time [s]
    # 预测时域
    N = N_pre  # prediction horizon
    # USV的半径，用于画图
    rob_diam = 0.2  # [m]
    # USV允许的最大速度
    v_max = 5
    # USV允许的最大角速度
    omega_max = np.pi * 0.8

    # 按照casadi的格式，声明状态变量，x，y，theta航向角
    x = ca.SX.sym('x')
    y = ca.SX.sym('y')
    theta = ca.SX.sym('theta')
    states = ca.vertcat(x, y)
    states = ca.vertcat(states, theta)  # 构建小车状态向量
    # n_states表示状态变量的个数
    n_states = states.size()[0]

    # 按照casadi的格式，声明控制变量，v，omega角速度
    v = ca.SX.sym('v')
    omega = ca.SX.sym('omega')
    controls = ca.vertcat(v, omega)
    # n_controls表示控制量的个数
    n_controls = controls.size()[0]

    # 按照casadi的模板定义状态方程
    ## rhs
    rhs = ca.vertcat(v * ca.cos(theta), v * ca.sin(theta))  ### 这里加了个负号，就尼玛尼普
    rhs = ca.vertcat(rhs, omega)

    ## function,[]输入，[]输出
    f = ca.Function('f', [states, controls], [rhs], ['input_state', 'control_input'], ['rhs'])

    # 产生矩阵
    ## for MPC
    U = ca.SX.sym('U', n_controls, N)

    X = ca.SX.sym('X', n_states, N + 1)  # 通常比控制多1

    P = ca.SX.sym('P', n_states + n_states)  # 在这里每次只需要给定当前/初始位置和目标终点位置

    # 优化目标的权重
    ### define
    Q = np.array([[5.0, 0.0, 0.0], [0.0, 5.0, 0.0], [0.0, 0.0, 0.1]])
    R = np.array([[0.5, 0.0], [0.0, 0.05]])
    #### cost function
    obj = 0  #### cost
    g = []  # equal constrains
    g.append(X[:, 0] - x_c[:3])
    for i in range(N):
        ## 距离目标点状态的cost
        state_error = 5 * (X[0, i] - P[3]) ** 2 + 5 * (X[1, i] - P[4]) ** 2 + 0.1 * (X[2, i] - P[5]) ** 2
        ## 控制的变化的cost
        control_error = 0.5 * (U[0, i]) ** 2 + 0.05 * (U[1, i]) ** 2
        ## 靠近ref的cost
        if goal_num == 0:
            horizon_error = 0
        else:
            horizon_error = ((des[goal_num][1] - des[goal_num - 1][1]) * X[0, i] - (
                        des[goal_num][0] - des[goal_num - 1][0]) * X[1, i] + (
                                         des[goal_num][0] * des[goal_num - 1][1] - des[goal_num][1] * des[goal_num - 1][
                                     0])) ** 2 / ((des[goal_num][0] - des[goal_num - 1][0]) ** 2 + (
                        des[goal_num][1] - des[goal_num - 1][1]) ** 2)
        # obj = obj + ca.mtimes([(X[:, i] - P[3:]).T, Q, X[:, i] - P[3:]]) + ca.mtimes([U[:, i].T, R, U[:, i]])
        x_next_ = f(X[:, i], U[:, i]) * T + X[:, i]

        ## 尽可能满足运动学约束的cost
        dynamic_error = (X[0,i+1] - x_next_[0])**2 + (X[0,i+1] - x_next_[0])**2 + (X[1,i+1] - x_next_[1])**2
        obj = obj + 1*state_error + control_error + 30 * horizon_error + 10000*dynamic_error
        #g.append(X[:, i + 1] - x_next_)

    #### obsatcle constraints
    for i in range(N + 1):
        for j in range(len(obs[0])):
            obj = obj + 10000/ca.sqrt((X[0, i] - obs[0][j]) ** 2 + (X[1, i] - obs[1][j]) ** 2)
            #g.append(ca.sqrt((X[0, i] - obs[0][j]) ** 2 + (X[1, i] - obs[1][j]) ** 2))  # should be smaller als 0.0
    opt_variables = ca.vertcat(ca.reshape(U, -1, 1), ca.reshape(X, -1, 1))

    nlp_prob = {'f': obj, 'x': opt_variables, 'p': P, 'g': ca.vertcat(*g)}
    opts_setting = {'ipopt.max_iter': 500, 'ipopt.print_level': 0, 'print_time': 0, 'ipopt.acceptable_tol': 1e-8,
                    'ipopt.acceptable_obj_change_tol': 1e-6}

    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

    # 状态量和控制量的约束的范围
    lbg = []
    ubg = []
    lbx = []
    ubx = []
    lbg.append(0.0)
    lbg.append(0.0)
    lbg.append(0.0)
    ubg.append(0.0)
    ubg.append(0.0)
    ubg.append(0.0)
    # for _ in range(N + 1):
    #     for i in range(len(obs[0])):
    #         lbg.append(5)  # safe distance
    #         ubg.append(np.inf)
    for _ in range(N):
        lbx.append(0)
        lbx.append(-omega_max)
        ubx.append(v_max)
        ubx.append(omega_max)
    for _ in range(N + 1):
        lbx.append(-10000.0)
        lbx.append(-10000.0)
        lbx.append(-np.inf)
        ubx.append(10000.0)
        ubx.append(10000.0)
        ubx.append(np.inf)

    # Simulation
    x0 = np.array(x_c).reshape(-1, 1)  # initial state
    x0_ = x0.copy()
    x_m = np.zeros((n_states, N + 1))
    next_states = x_m.copy()
    xs = np.array(x_s).reshape(-1, 1)  # final state
    u0 = np.array([1, 0] * N).reshape(-1, 2).T  # np.ones((N, 2)) # controls

    ## start MPC
    ## 设置初始点和目标点(通过这里实现强制左转弯和右转弯和绕圈)
    c_p = np.concatenate((x0, xs))
    init_control = np.concatenate((u0.T.reshape(-1, 1), next_states.T.reshape(-1, 1)))
    t_ = time.time()
    res = solver(x0=init_control, p=c_p, lbg=lbg, lbx=lbx, ubg=ubg, ubx=ubx)
    estimated_opt = res['x'].full()  # the feedback is in the series [u0, x0, u1, x1, ...]
    u0 = estimated_opt[:N * n_controls].reshape(N, n_controls).T  # (n_controls, N)
    x_m = estimated_opt[N * n_controls:].reshape(N + 1, n_states).T  # [n_states, N]
    data_to_log = [x_m[0], x_m[1]]
    write_data()
    return x_m[0], x_m[1]


if __name__ == '__main__':
    '''
    ss = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ss.connect(('127.0.0.1', 50520))
    data = 'obsposition'
    data = data + '\r'
    ss.send(data.encode("utf-8"))
    dataa = ss.recv(1024).decode("utf-8")
    print()
    ss.close()
    '''
    target_num = 0
    had_arrive_the_dis = False
    if_arrived_current = False
    decide_angle = 0
    points_angle = []
    boat = [0, 0, 0, 0, 0]  # //x,z,speedx,speedz,rotation[1]
    # des = [[20,20,0],[20,60,math.pi/2],[20,110,math.pi/2],[20,140,math.pi/2],[20,180,math.pi/2],[20,240,math.pi/2],[20,280,math.pi/2],[150,150,0],[280,280,0],[280,20,0]]
    destination = [130, 20]
    des = [[20,20,0],[130,20,0]]
    target_pointx = []  # 自己随便设置的一些目标点
    target_pointy = []
    ki = 0.2
    goal_num = 0
    file_path = r'data_log.csv'
    # log_data = open(file_path, 'a')
    data_to_log = []
    obs = [[55,55,55,55,55,85,85,85,85,85], [20,25,30,35,40,20,15,10,5,0]]
    #obs =[[], []]
    #target_pointx = [20, 20]  # 自己随便设置的一些目标点
    # target_pointy = [30, 40]

    # target_pointx,target_pointy = planning([0,0,1],[20,280,1],15)
    # plt.plot([20],[280],'*')
    # plt.plot(target_pointx,target_pointy,'o')
    # plt.show()
    # data_to_log = ["ki", "power_max", "speed_max", "target_num", "detal1", "et" , "decide_angle", "miss", "max_miss", "speed", "left", "right", "boat_angle"]
    # write_data()
    # data_to_log[0] = ki
    max_miss = 0
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 连接仿真端
    s.connect(('127.0.0.1', 50520))
    s.send('setall 0\r'.encode("utf-8"))
    s.recv(1024).decode("utf-8")
    s.send('reset\r'.encode("utf-8"))
    s.recv(1024).decode("utf-8")



    t1 = Thread(target=send_and_recv)
    t1.start()
    t2 = Thread(target=plot_ponit)
    t2.start()
    print('接收和发送线程已经启动\n')
    while not had_arrive_the_dis:
        print("斯坦利在循迹\n")
        sleep(2)
    t2.join()
    t1.join()
    s.close()