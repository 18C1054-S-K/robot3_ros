#!/usr/bin/env python3

import time
import math
import numpy as np
import rospy
from std_msgs.msg import Bool, Float32, Float32MultiArray
from robot3_18c1054.msg import HandClose
from robot3_18c1054.srv import GetHandState, GetHandStateResponse, GetInitTime, GetInitTimeResponse

class ArmControllerNode():
	DELTA_TIME = 0.1
	DELTA_TIME_HAND = 0.013
	EPSIRON = 0.0001
	LINK_LEN = [0.1, 0.3, 0.25, 0.1]

	#p_gain = a^2, d_gain = 2a
	DEFAULT_P_GAIN = 64.0
	DEFAULT_D_GAIN = 16.0
	MAX_ANG_V = 2.5
	NEAR_ANGLE = 1.2 #if sum abs(angular displacement) < NEAR_ANGLE , let us call it "near"

	target_ang_angv = [0.0]*12
	current_ang_angv = [0.0]*12
	deff_ang_angv = [0.0]*12
	#Runge-Kuta
	k = [[0.0]*12, [0.0]*12, [0.0]*12, [0.0]*12]
	#PD gain
	p_gain = 0.0
	d_gain = 0.0
	is_reached_target = True	

	is_hand_close = False
	is_hand_reserved = False
	hand_countdown = 0.0
	hand_close_time = 0.0

	debug_count = 0

	init_time = 0.0
	
	def __init__(self):
		self.init_time = time.time()

		self.server_getinittime = rospy.Service('get_init_time', GetInitTime, self.teach_init_time)
		self.server_getstate = rospy.Service('get_hand_state', GetHandState, self.teach_hand_state)
		self.pub_arm = rospy.Publisher('arm_ang_angv', Float32MultiArray, queue_size=10)
		self.pub_hand = rospy.Publisher('hand_close', HandClose, queue_size=10)
		self.sub_target = rospy.Subscriber('target_arm_state', Float32MultiArray, self.update_target)
		self.sub_hand = rospy.Subscriber('hand_close_reserve', Float32, self.reserve_hand)
		self.sub_test = rospy.Subscriber('ang_debug',Float32MultiArray,self.update_target_debug)
		self.timer_arm = rospy.Timer(rospy.Duration(self.DELTA_TIME), self.update_arm)
		self.timer_hand = rospy.Timer(rospy.Duration(self.DELTA_TIME_HAND), self.update_hand)


	def teach_init_time(self, req):
		s = math.ceil(self.init_time) #second
		lower = self.init_time - s #lower
		m = s // 60 #minute
		s -= m * 60
		h = m // 60 #hour
		m -= h * 60
		d = h // 24 #day
		h -= d*24
		M = d // 30 #month
		d -= M * 30
		y = M // 12 #year
		M -= y*12
		return GetInitTimeResponse(y, M, d, h, m, s, lower)


	def teach_hand_state(self, req):
		arr = self.FK(self.current_ang_angv)
		arr_re = [arr[i] for i in range(15)]
		return GetHandStateResponse(arr_re)


	def update_target_debug(self, msg):
		if len(msg.data) >= 12:
			self.is_reached_target = False
			for i in range(12):
				self.target_ang_angv[i] = msg.data[i]
		elif len(msg.data) >= 6:
			self.is_reached_target = False
			for i in range(6):
				self.target_ang_angv[i] = msg.data[i]
				self.target_ang_angv[i+6] = 0.0


	def update_target(self, msg):
		if len(msg.data) >= 12:
			self.is_reached_target = False
			target = self.IK(msg.data)
			if len(target) >= 12:
#				print('-----updated target----- : ',target)
				for i in range(12):
					self.target_ang_angv[i] = target[i]
			else:
#				print('-----TargetError----- : ',target)
				self.update_hand(False)
				self.is_hand_reserved = False


	def update_arm(self, event):
		if not rospy.is_shutdown():
			if self.is_reached_target:
				for i in range(6):
					self.deff_ang_angv[i] = 0.0
					self.deff_ang_angv[i+6] = self.current_ang_angv[i+6]
			else:
				for i in range(12):
					self.deff_ang_angv[i] = self.current_ang_angv[i] - self.target_ang_angv[i]
			need_to_update = False
			for i in range(12):
				need_to_update = need_to_update or abs(self.deff_ang_angv[i])> self.EPSIRON

			if need_to_update == True:
				#determine PD gain
				sum_abs_ang = 0.0
				for i in range(6):
					sum_abs_ang = abs(self.deff_ang_angv[i])
				if sum_abs_ang < self.NEAR_ANGLE:
					self.p_gain = self.DEFAULT_P_GAIN
					self.d_gain = self.DEFAULT_D_GAIN
				else:
					too_fast = False
					for i in range(6):
						too_fast = too_fast or abs(self.current_ang_angv[i+6]) >= self.MAX_ANG_V
					if too_fast:
						self.p_gain = 0.0
						self.d_gain = 0.0
					else:
						self.p_gain = self.DEFAULT_P_GAIN
						self.d_gain = 0.0
							
				#feed back controll
				temp = [0.0]*12
				for i in range(4):
					for j in range(12):
						self.k[i][j] = 0.0
				#calc k1
				for i in range(6):
					self.k[0][i] += self.deff_ang_angv[i + 6]
					self.k[0][i + 6] -= self.p_gain * self.deff_ang_angv[i]
					self.k[0][i + 6] -= self.d_gain * self.deff_ang_angv[i + 6]
				#calc k2
				for i in range(12):
					temp[i] = self.deff_ang_angv[i] + 0.5 * self.DELTA_TIME * self.k[0][i]
				for i in range(6):
					self.k[1][i] += temp[i + 6]
					self.k[1][i + 6] -= self.p_gain * temp[i]
					self.k[1][i + 6] -= self.d_gain * temp[i + 6]
				#calc k3
				for i in range(12):
					temp[i] = self.deff_ang_angv[i] + 0.5 * self.DELTA_TIME * self.k[1][i]
				for i in range(6):
					self.k[2][i] += temp[i + 6]
					self.k[2][i + 6] -= self.p_gain * temp[i]
					self.k[2][i + 6] -= self.d_gain * temp[i + 6]
				#calc k_4
				for i in range(12):
					temp[i] = self.deff_ang_angv[i] + self.DELTA_TIME * self.k[2][i]
				for i in range(6):
					self.k[3][i] += temp[i + 6]
					self.k[3][i + 6] -= self.p_gain * temp[i]
					self.k[3][i + 6] -= self.d_gain * temp[i + 6]
				#update
				deff_pre = 0.0
				deff_now = 0.0
				for i in range(6):
					temp[i] = self.current_ang_angv[i+6]
					temp[i] = 0.0
				for i in range(12):
					temp[i] += (self.k[0][i] + 2*self.k[1][i] + 2*self.k[2][i] + self.k[3][i]) / 6
					temp[i] *= self.DELTA_TIME
					self.current_ang_angv[i] += temp[i]

					#check arm is reached target
					deff_pre += self.deff_ang_angv[i]**2
					deff_now += temp[i]**2
				if deff_now <= 6.0*self.EPSIRON*self.EPSIRON:
					self.is_reached_target = True
				elif deff_now > deff_pre and deff_pre < self.EPSIRON:
					self.is_reached_target = True
				#publish arm's joint's angle & angular velocity
				arr_pub = Float32MultiArray(data=self.current_ang_angv)
				self.pub_arm.publish(arr_pub)


	def reserve_hand(self, msg):
		self.is_hand_reserved = True
		self.hand_close_time = self.init_time + msg.data


	def update_hand(self, event):
		if not rospy.is_shutdown():
			if self.is_hand_reserved:
				time_now = time.time()
				if self.hand_close_time - time_now < self.DELTA_TIME_HAND/2:
					hc_pub = HandClose()
					hc_pub.close = True
					hc_pub.time_stamp = time_now - self.init_time
					self.pub_hand.publish(hc_pub)

					self.hand_countdown = 3.0 #cout down for open hand
					self.is_hand_close = True
					self.is_hand_reserved = False
			else:
				if self.is_hand_close:
					if self.hand_countdown <= 0.0:
						hc_pub = HandClose()
						hc_pub.close = False
						hc_pub.time_stamp = 0.0 #no use, anything OK
						self.pub_hand.publish(hc_pub)

						self.is_hand_close = False
					else:
						self.hand_countdown -= self.DELTA_TIME_HAND


	def index_to_axisname(self, index):
		if index == 0 or index == 2 or index == 4:
			return 'x'
		elif index == 1 or index == 3:
			return 'y'
		elif index == 5:
			return 'z'
		else:
			return 'err'


	#calcurate rotation matrix
	def rot_mat(self, axis_name, cos_, sin_):
		if axis_name == 'x':
			R = np.matrix([[1.0, 0.0, 0.0], [0.0,cos_,-sin_], [0.0,sin_,cos_]])
			return R
		elif axis_name == 'y':
			R = np.matrix([[cos_,0.0,sin_], [0.0, 1.0, 0.0], [-sin_,0.0,cos_]])
			return R
		elif axis_name == 'z':
			R = np.matrix([[cos_,-sin_,0.0], [sin_,cos_,0.0], [0.0, 0.0, 1.0]])
			return R
		else:
			R = np.eye(3)
			return R


	#calcurate d(rotation matrix)/dt devided by angular velocity
	def d_rot_mat(self, axis_name, cos_, sin_):
		if axis_name == 'x':
			dR = np.matrix([[0.0]*3, [0.0,-sin_,-cos_], [0.0,cos_,-sin_]])
			return dR
		elif axis_name == 'y':
			dR = np.matrix([[-sin_,0.0,cos_], [0.0]*3, [-cos_,0.0,-sin_]])
			return dR
		elif axis_name == 'z':
			dR = np.matrix([[-sin_,-cos_,0.0], [cos_,-sin_,0.0], [0.0]*3])
			return dR
		else:
			dR = np.zeros((3,3))
			return dR


	#front kinematics
	def FK(self,ang_angv):
		l = [np.array([[0.0], [self.LINK_LEN[i]], [0.0]]) for i in range(4)]

		cos_ = [math.cos(ang_angv[i]) for i in range(6)]
		sin_ = [math.sin(ang_angv[i]) for i in range(6)]
		R = [self.rot_mat(self.index_to_axisname(i),cos_[i],sin_[i]) for i in range(6)]
		dR = [self.d_rot_mat(self.index_to_axisname(i),cos_[i],sin_[i]) for i in range(6)]
		for i in range(6):
			dR[i] = ang_angv[i+6]*dR[i]

		att = R[0] @ R[1] @ R[2] @ R[3] @ R[4] @ R[5]

		att_v  = dR[0] @ R[1] @ R[2] @ R[3] @ R[4] @ R[5]
		att_v += R[0] @ dR[1] @ R[2] @ R[3] @ R[4] @ R[5]
		att_v += R[0] @ R[1] @ dR[2] @ R[3] @ R[4] @ R[5]
		att_v += R[0] @ R[1] @ R[2] @ dR[3] @ R[4] @ R[5]
		att_v += R[0] @ R[1] @ R[2] @ R[3] @ dR[4] @ R[5]
		att_v += R[0] @ R[1] @ R[2] @ R[3] @ R[4] @ dR[5]

		pos = np.zeros((3,1))
		pos += l[0]
		pos += R[0] @ l[1]
		pos += R[0] @ R[1] @ R[2] @ l[2]
		pos += att @ l[3]

		pos_v = np.zeros((3,1))
		pos_v += dR[0] @ l[1]
		pos_v += dR[0] @ R[1] @ R[2] @ l[2]
		pos_v += R[0] @ dR[1] @ R[2] @ l[2]
		pos_v += R[0] @ R[1] @ dR[2] @ l[2]
		pos_v += att_v @ l[3]

		arr = [0.0]*24
		for i in range(3):
			arr[i] = pos[i,0]
			arr[i+3] = pos_v[i,0]
			for j in range(3):
				arr[i*3+j+6] = att[i,j]
				arr[i*3+j+15] = att_v[i,j]
		return arr


	#inverse kinematics
	def IK(self, hand_state):
		pos = [0.0]*3
		pos_v = [0.0]*3
		att = np.zeros((3,3)) #rotation matrix
		att_v = np.zeros((3,3))
		for i in range(3):
			pos[i] = hand_state[i]
			pos_v[i] = hand_state[i+3]
			for j in range(3):
				att[i,j] = hand_state[i*3+j+6]
				att_v[i,j] = hand_state[i*3+j+15]
		speed_zero = True
		epsiron = 0.0005
		for i in range(3):
			speed_zero = speed_zero and (pos_v[i]<epsiron) and (pos_v[i]>-epsiron)
			for j in range(3):
				speed_zero = speed_zero and (att_v[i,j]<epsiron) and (att_v[i,j]>-epsiron)


		#calc angle
		ang = [0.0]*6
		temp = 0.0

		#calc joint_1~3 angle
		#wrist position(relative pos from joint_1)
		wrist = [0, 0, 0]
		wrist[0] = pos[0] - self.LINK_LEN[3]*att[0,1]
		wrist[1] = pos[1] - self.LINK_LEN[3]*att[1,1] - self.LINK_LEN[0]
		wrist[2] = pos[2] - self.LINK_LEN[3]*att[2,1]
		#calc joint_3 angle
		for i in range(3):
			temp += wrist[i]**2
		ang[2] = self.LINK_LEN[1]**2 + self.LINK_LEN[2]**2 - temp
		ang[2] /= 2.0 * self.LINK_LEN[1] * self.LINK_LEN[2]
		if ang[2] > 1 or ang[2] < -1:
			return [3]
		else:
			ang[2] = math.pi - math.acos(ang[2])
		#calc joint_2 angle
		temp = self.LINK_LEN[2] * math.sin(ang[2])
		if temp == 0:
			ang[1] = 0
		else:
			ang[1] = wrist[0] / temp
			if ang[1] > 1 or ang[1] < -1:
				return [2]
			else:
				ang[1] = math.asin(ang[1])
		#calc joint_1 angle
		temp_z = temp * math.cos(ang[1])
		temp_y = self.LINK_LEN[1] + self.LINK_LEN[2]*math.cos(ang[2])
		ang[0] = temp_z * wrist[2] + temp_y * wrist[1]
		ang[0] /= wrist[2]**2 + wrist[1]**2
		if ang[0] > 1 or ang[0] < -1:
			return [1]
		else:
			ang[0] = math.acos(ang[0])
			if wrist[2] < temp_z:
				ang[0] *= -1

		#calc joint_4~5 angle
		R = self.rot_mat('x', math.cos(-ang[0]), math.sin(-ang[0]))@att
		R = self.rot_mat('y', math.cos(-ang[1]), math.sin(-ang[1]))@R
		R = self.rot_mat('x', math.cos(-ang[2]), math.sin(-ang[2]))@R
		#is cos(joint_5 angle) < 0 
		#joint_6 angle in [-pi/2,pi/2] so cos5 can determine + or -
		cos5_minus = False
		#calc joint_6 angle
		temp_y = R[1,0]
		temp_x = R[1,1]
		if temp_y == 0 and temp_x == 0:
			ang[5] = 0
		else:
			ang[5] = math.atan2(temp_y, temp_x)
		if ang[5] > math.pi / 2:
			ang[5] -= math.pi
		elif ang[5] < -math.pi / 2:
			ang[5] += math.pi
		if temp_x < 0:
			cos5_minus = True
		#calc joint_4 angle
		temp_y = R[0, 2]
		temp_x = R[2, 2]
		if cos5_minus == True:
			temp_y *= -1.0
			temp_x *= -1.0
		if temp_y == 0 and temp_x == 0:
			ang[3] = 0
		else:
			ang[3] = math.atan2(temp_y, temp_x)
		#calc joint_5 angle
		ang[4] = -R[1,2]
		if ang[4] < -1 or ang[4] > 1:
			return [5]
		ang[4] = math.asin(ang[4])
		if cos5_minus == True:
			if ang[4] > 0:
				ang[4] = math.pi - ang[4]
			elif ang[4] < 0:
				ang[4] = -math.pi - ang[4]

		#calc angular velocity
		ang_v = [0.0]*6
		if speed_zero == False:
			coef_mat = np.zeros((6,6))
			const_v = np.zeros((6,1))
			#calc const_v
			for i in range(3):
				const_v[i,0] = pos_v[i]
				for j in range(3):
					const_v[i+3,0] += att_v[i,j]
			cos_=[math.cos(ang[i]) for i in range(6)]
			sin_=[math.sin(ang[i]) for i in range(6)]
			#calc coef_mat's row 1~3,column 1
			temp_M = self.d_rot_mat('x',cos_[0],sin_[0])
			temp_v = temp_M @ np.array([[0.0],[self.LINK_LEN[1]],[0.0]])
			temp_M = temp_M @ self.rot_mat('y',cos_[1],sin_[1]) @ self.rot_mat('x',cos_[2],sin_[2])
			temp_v += temp_M @ np.array([[0.0],[self.LINK_LEN[2]],[0.0]])
			temp_M = temp_M @ self.rot_mat('y',cos_[3],sin_[3]) @ self.rot_mat('x',cos_[4],sin_[4]) @ self.rot_mat('z',cos_[5],sin_[5])
			temp_v += temp_M @ np.array([[0.0],[self.LINK_LEN[3]],[0.0]])
			for i in range(3):
				coef_mat[i,0] = temp_v[i,0]
			#calc coef_mat's row 1~3,column 2
			temp_M = self.rot_mat('x',cos_[0],sin_[0]) @ self.d_rot_mat('y',cos_[1],sin_[1]) @ self.rot_mat('x',cos_[2],sin_[2])
			temp_v = temp_M @ np.array([[0.0],[self.LINK_LEN[2]],[0.0]])
			temp_M = temp_M @ self.rot_mat('y',cos_[3],sin_[3]) @ self.rot_mat('x',cos_[4],sin_[4]) @ self.rot_mat('z',cos_[5],sin_[5])
			temp_v += temp_M @ np.array([[0.0],[self.LINK_LEN[3]],[0.0]])
			for i in range(3):
				coef_mat[i,1] = temp_v[i,0]
			#calc coef_mat's row 1~3,column 3
			temp_M = self.rot_mat('x',cos_[0],sin_[0]) @ self.rot_mat('y',cos_[1],sin_[1]) @ self.d_rot_mat('x',cos_[2],sin_[2])
			temp_v = temp_M @ np.array([[0.0],[self.LINK_LEN[2]],[0.0]])
			temp_M = temp_M @ self.rot_mat('y',cos_[3],sin_[3]) @ self.rot_mat('x',cos_[4],sin_[4]) @ self.rot_mat('z',cos_[5],sin_[5])
			temp_v += temp_M @ np.array([[0.0],[self.LINK_LEN[3]],[0.0]])
			for i in range(3):
				coef_mat[i,2] = temp_v[i,0]
			#calc coef_mat's row 1~3,column 4~6
			for i in range(3,6):
				temp_v = np.array([[0.0],[self.LINK_LEN[3]],[0.0]])
				for j in range(6):
					if i == 5-j:
						temp_v = self.d_rot_mat(self.index_to_axisname(5-j),cos_[5-j],sin_[5-j]) @ temp_v
					else:
						temp_v = self.rot_mat(self.index_to_axisname(5-j),cos_[5-j],sin_[5-j]) @ temp_v
				for j in range(3):
					coef_mat[j,i] = temp_v[j,0]
			#calc coef_mat's row 4~6,column 1~6
			for i in range(6):
				dR = self.d_rot_mat(self.index_to_axisname(i),cos_[i],sin_[i])
				for j in range(3):
					for k in range(3):
						coef_mat[j+3,i] += dR[j,k]
			#solve equatation to calc ang_v
			try:
				ang_v = np.linalg.solve(coef_mat,const_v)
			except np.linalg.LinAlgError:
				ang_v = [0.0]*6

		#return
		arr = [0.0]*12
		for i in range(6):
			arr[i] = ang[i]
			arr[i+6] = ang_v[i]
		return arr

def main():
	rospy.init_node('arm_controller', anonymous=True)
	node = ArmControllerNode()
	rospy.spin()


if __name__ == '__main__':
	main()
