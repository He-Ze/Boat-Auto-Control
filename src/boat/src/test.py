import rospy
import math
import csv
import time
import struct
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray
import casadi as ca
import casadi.tools as ca_tools
import numpy as np
from threading import Thread
import xlwt


def write_data():
	global file_path, data_to_log
	with open(file_path, "a+") as log_file:
		writer = csv.writer(log_file)
		writer.writerow(data_to_log)


class mpc_stanly_com(object):
	# 初始化和接受信息的部分
	def __init__(self, arg):
		self.pub_control = rospy.Publisher('vtg',Vector3,queue_size=1000)
		self.flag = 0
		self.w_last = 0
		print('init has been done\n')

	def listener(self):
		rospy.Subscriber("unionstrong/gpfpd",NavSatFix,self.get_gps)
		print('start to listen the msg and MPCing\n')
		rospy.spin()


	def gps_to_mkt(longitude,latitude):
		x = longitude*20037508.34/180
		y = math.log(math.tan((90+latitude)*math.pi/360))/(math.pi/(180))
		y = y*20037508.34/180
		return x,y

	def if_change_ref(x1,y1,x2,y2):
		error = 0
		thr = 10000
		for i in range(len(x1)):
			error = error + (x1[i]-x2[i])**2 + (y1[i]-y2[i])**2
		if error < thr:
			return True
		else:
			return False

	def get_gps(self, msg):
		print("strat recv gps\n")
		global boat,had_arrive_the_dis, destination, target_pointx, target_pointy, des, target_num, goal_num, if_arrived_current
		heading = msg.position_covariance[1]
		if boat[3] == msg.position_covariance[0]:
			print('收到相同的时间戳\n')
		boat[0],boat[1] = gps_to_mkt(msg.longitude,msg.latitude)
		boat[2] = heading/180*math.pi #弧度制
		boat[3] = msg.position_covariance[0] #对应的时间戳
		data_to_log[0] = boat[0]
		data_to_log[1] = boat[1]
		data_to_log[2] = boat[2]
		data_to_log[3] = boat[3]
		print ("gps pos:",boat[0],boat[1],boat[2],boat[3])
		print ("receive the msg\n")
		if not had_arrive_the_dis:
			if self.flag%4 == 0:
				temp_target_pointx,temp_target_pointy = planning([boat[0], boat[1], temp], des[goal_num], 20)
				if if_change_ref(temp_target_pointx,temp_target_pointy,target_pointx,target_pointy) :
					target_pointx = temp_target_pointx
					target_pointy = temp_target_pointy
			if get_dis([boat[0], boat[1]], des[goal_num][:2]) < 15 and len(des) > 1:
				goal_num += 1
				target_pointx, target_pointy = planning([boat[0], boat[1], temp], des[goal_num], 20)
			if get_dis([boat[0], boat[1]], destination) < 5:
				had_arrive_the_dis = True

			self.flag +=1
			self.point_pre_process()
			self.find_nearest_point()
			self.find_deci_angle()
			self.change_motor()
			write_data()
		return

	### 这里与原来的代码不相同
	def get_angle(point_pre, point_post):
		temp_x = point_post[0] - point_pre[0]
		temp_y = point_post[1] - point_pre[1]
		temp_angle = math.atan2(temp_x, temp_y)

		# if temp_angle >= 0:
        # 	return temp_angle - math.pi
		# else:
        # 	return math.pi + temp_angle

		return temp_angledd


	def get_angle_dis(angle1, angle2):  # 逆时针方向为负，顺时针为正
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
				target_num = i + 8
			i = i + 1
		print("Nearset point is", target_num, temp_dis)
		data_to_log[5] = target_num
		return


	def find_deci_angle():
		global boat, target_num, points_angle, ki, decide_angle, data_to_log,max_miss
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
		data_to_log[6] = ki
		data_to_log[7] = delta1
		#data_to_log[5] = et
		data_to_log[9] = decide_angle
		data_to_log[8] = et
		data_to_log[12] = e_dis
		data_to_log[13] = max_miss
		print("decide_angle is ", decide_angle)
		return


	def change_motor():
		global s, decide_angle, had_arrive_the_dis, data_to_log,if_arrived_current
		current_cmd = Vector3()
		current_cmd.x = 0.4
		if decide_angle >= 0:
			w = math.sqrt(decide_angle/math.pi)
		else:
			w = -math.sqrt(math.fabs(decide_angle/math.pi)) #### 这里也不一样
		if math.fabs(w)>1:
			vtg.y = w/math.fabs(w)
		else:
			vtg.y = w
		if vtg.y - self.w_last > 0.4:
			vtg.y = self.w_last + 0.4
		elif vtg.y - self.w_last < -0.4:
			vtg.y = self.w_last - 0.4
		else:
			vtg.y = vtg.y
		self.w_last = vtg.y
		self.pub_control.publish(current_cmd)
		
'''
	def plot_ponit():
		global target_pointx,target_pointy,boat,target_num
		plt.ion()
		plt.axis('equal')
		while True:
			plt.clf()
			plt.plot(boat[0],[boat[1]],'*')
			plt.plot(target_pointx[target_num],target_pointy[target_num],'o')
			plt.plot(target_pointx[target_num+2],target_pointy[target_num+2],'^')
			plt.plot(target_pointx,target_pointy,'.')
			plt.pause(0.1)
			plt.ioff()
'''  


def planning(x_c,x_s,N_pre):
	global data_to_log,goal_num,des,had_arrive_the_dis,target_pointx,target_pointy
	# 取样时间，用于估计未来状态的
	T = 0.3  # sampling time [s]
	# 预测时域
	N = N_pre  # prediction horizon
	# USV的半径，用于画图
	rob_diam = 0.2  # [m]
	# USV允许的最大速度
	v_max = 5
	# USV允许的最大角速度
	omega_max = np.pi*0.8

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
	g.append(X[:, 0] - P[:3])
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
        obj = obj + state_error + control_error + 30 * horizon_error + 10000*dynamic_error
        #g.append(X[:, i + 1] - x_next_)

	#### obsatcle constraints
	for i in range(N + 1):
		for j in range(len(obs[0])):
			g.append(ca.sqrt((X[0, i] - obs[0][j]) ** 2 + (X[1, i] - obs[1][j]) ** 2))  # should be smaller als 0.0
	opt_variables = ca.vertcat(ca.reshape(U, -1, 1), ca.reshape(X, -1, 1))

	nlp_prob = {'f': obj, 'x': opt_variables, 'p': P, 'g': ca.vertcat(*g)}
	opts_setting = {'ipopt.max_iter': 500, 'ipopt.print_level': 0, 'print_time': 0, 'ipopt.acceptable_tol': 1e-8,
					'ipopt.acceptable_obj_change_tol': 1e-6}

	solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

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
	for _ in range(N + 1):
		for i in range(len(obs[0])):
			lbg.append(5)  # safe distance
			ubg.append(np.inf)
	for _ in range(N):
		lbx.append(0)
		lbx.append(-omega_max)
		ubx.append(v_max)
		ubx.append(omega_max)
	for _ in range(N + 1):
		lbx.append(-np.inf)
		lbx.append(-np.inf)
		lbx.append(-np.inf)
		ubx.append(np.inf)
		ubx.append(np.inf)
		ubx.append(np.inf)

	# Simulation
	x0 = np.array(x_c).reshape(-1, 1)  # initial state
	x0_ = x0.copy()
	x_m = np.zeros((n_states, N + 1))
	next_states = x_m.copy()
	xs = np.array(x_s).reshape(-1, 1)
		temp_dis = 10000
		while i < len(target_pointx) - 11:
			dis = get_dis([target_pointx[i], target_pointy[i]], [boat[0], boat[1]])
			if temp_dis > dis:
				temp_dis = dis  # final state
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
	data_to_log[4] = goal_num
	#data_to_log = [x_m[0], x_m[1]]
	#write_data()
	return x_m[0],x_m[1]


if __name__ == '__main__':
	# 变量参数
	boat = [0, 0, 0, 0] #pointx,pointy,heading,time
	target_num = 0
	had_arrive_the_dis = False
	if_arrived_current = False
	decide_angle = 0
	points_angle = []
	destination = [0,0]
	des = [[20,20,0],[20,280,3*math.pi/4],[150,150,math.pi/4],[280,280,math.pi],[280,20,math.pi]]
	target_pointx = []  # 自己随便设置的一些目标点
	target_pointy = []
	ki = 0.2
	goal_num = 0
	file_path = r'data_log.csv'  #设置log文件位置
	data_to_log = ["pointx","pointy","angle","time","goal_num","target_num","ki","heading_dis","et","decide_angle","speed","w","miss","max_miss"] #存储文件的内容
	obs = [[],[]]
	max_miss = 0

	rospy.init_node("Mpc_Stanly_com",anonymous=True)
	print("thread create\n")
	mpc_stanly = mpc_stanly_com()
	mpc_stanly.listener()

'''
import paho.mqtt.client as mqtt
import struct



def on_connect(client,userdata,flags,rc):
    print("connected with reult code " + str(rc))
    client.subscribe('/pose')
    print("connect success")

def on_message(client,userdata,msg):
    topic = '/ctrl'
    v=0.8
    r=-1
    p=50
    data = struct.pack(">ffB",v,r,p)
    client.publish(topic,payload=data)
    if msg.topic == '/pose' :
        a,b = struct.unpack('>ff',msg.payload)
        print("pose")

client = mqtt.Client()
client.on_connect=on_connect
client.on_message=on_message
client.connect("192.168.1.230",1883,600)
client.loop_forever()
'''
