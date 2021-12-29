#!/usr/bin/env python3

import rospy
import numpy as np
import math
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray, Bool, Header
from test.msg import HandClose
from test.srv import GetHandState, GetHandStateResponse, GetInitTime, GetInitTimeResponse

class VisualizerNode():
	GRAVITY = 9.8
	JOINT_NUM = 6
	DELTA_TIME = 0.05

	arm_ang = [0.0]*6
	is_hand_close = False
	hand_state = [0.0]*6
	hand_ang_close = 0.0
	hand_ang_open = 0.8

	is_shooted = False
	shoot_timestamp = time.time()
	is_ball_flying = False
	shooter_state = [0.0, -1.0, 0.0, 0.0]
	ball_state = [0.0]*6
	ball_initial_state = [0.0]*6

	init_time = 0.0

	def __init__(self):
		#get init_time
		rospy.wait_for_service('get_init_time')
		try:
			get_init_time = rospy.ServiceProxy('get_init_time', GetInitTime)
			resp = get_init_time()
			self.init_time = resp.year
			self.init_time = self.init_time*12 + resp.month
			self.init_time = self.init_time*30 + resp.day
			self.init_time = self.init_time*24 + resp.hour
			self.init_time = self.init_time*60 + resp.minute
			self.init_time = self.init_time*60 + resp.second
			self.init_time += resp.lower
		except rospy.ServiceException:
			print('service err in visualizer')
			self.init_time = time.time()
		
		self.pub_viz = rospy.Publisher('joint_states', JointState, queue_size=10)
		self.sub_arm = rospy.Subscriber('arm_ang_angv', Float32MultiArray, self.update_arm)
		self.sub_hand = rospy.Subscriber('hand_close', HandClose, self.update_hand)
		self.sub_shooter = rospy.Subscriber('shooter_state', Float32MultiArray, self.update_shooter)
		self.sub_shoot_1 = rospy.Subscriber('ball_initial_state', Float32MultiArray, self.shoot)
		self.timer = rospy.Timer(rospy.Duration(self.DELTA_TIME), self.redisp)


	def update_arm(self, msg):
		for i in range(self.JOINT_NUM):
			self.arm_ang[i] = msg.data[i]


	def update_hand(self, msg):
		self.is_hand_close = msg.close
		if self.is_hand_close:
			count = (msg.time_stamp + self.init_time) - self.shoot_timestamp
			for i in range(3):
				self.ball_state[i+3] = self.ball_initial_state[i+3]
				self.ball_state[i] = self.ball_initial_state[i]
				self.ball_state[i] += count * self.ball_initial_state[i+3]
			self.ball_state[4] -= self.GRAVITY * count
			self.ball_state[1] -= (self.GRAVITY/2.0)*(count**2)
			self.check_catch()

#			self.is_ball_flying = False


	SAFE_DIST_X = 0.02
	SAFE_DIST_Y = 0.05
	SAFE_DIST_Z = 0.05
	SAFE_COS_ANG_Z = 0.75
	SAFE_COS_ANG_Y = 0.5
	def check_catch(self):
		rospy.wait_for_service('get_hand_state')
		try:
			get_hand_state = rospy.ServiceProxy('get_hand_state', GetHandState)
			resp = get_hand_state()

			hand_pos = np.zeros((3,1))
			hand_pos_v = np.zeros((3,1))
			hand_att = np.zeros((3,3))
			for i in range(3):
				hand_pos[i,0]=resp.hand_state[i]
				hand_pos_v[i,0]=resp.hand_state[i+3]
				for j in range(3):
					hand_att[i,j]=resp.hand_state[i*3+j+6]

			ball_pos = np.zeros((3,1))
			ball_pos_v = np.zeros((3,1))
			for i in range(3):
				ball_pos[i,0] = self.ball_state[i]
				ball_pos_v[i,0] = self.ball_state[i+3]

			print('--------------------------')
			rel_p = ball_pos - hand_pos
			rel_p = hand_att.T @ rel_p
			print("ball's relative position from hand :")
			print(rel_p)
			if abs(rel_p[0,0]) < self.SAFE_DIST_X and abs(rel_p[1,0]) < self.SAFE_DIST_Y and abs(rel_p[2,0]) < self.SAFE_DIST_Z:
				rel_v = ball_pos_v - hand_pos_v
				rel_v = hand_att.T @ rel_v
				v_norm = np.linalg.norm(rel_v)
				if (rel_v[2,0]>v_norm*self.SAFE_COS_ANG_Z or rel_v[2,0]<v_norm*self.SAFE_COS_ANG_Z) and rel_v[1,0]<v_norm*self.SAFE_COS_ANG_Y:
					self.is_ball_flying = False
			if self.is_ball_flying and self.is_shooted:
				print('ball catch : miss')
			elif self.is_shooted:
				print('ball catch : success')
			print('--------------------------')
		except rospy.ServiceException: pass


	def update_shooter(self, msg):
		for i in range(4):
			self.shooter_state[i] = msg.data[i]


	def shoot(self, msg):
		self.shoot_timestamp = msg.data[6] + self.init_time
		self.is_shooted = True
		self.is_ball_flying = True
		for i in range(6):
			self.ball_initial_state[i]=msg.data[i]


	def redisp(self, event):
		if not rospy.is_shutdown():
			j_s = JointState()
			j_s.header = Header()
			j_s.header.stamp = rospy.Time.now()
			j_s.name = ['hand_1','hand_2','shooter_x','shooter_z','shooter_roll','shooter_pitch','ball_x','ball_y','ball_z', 'joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
			arr = [0.0]*(9+self.JOINT_NUM)
			#arm
			for i in range(self.JOINT_NUM):
				arr[i+9] = self.arm_ang[i]
			#hand
			if self.is_hand_close:
				arr[0]=self.hand_ang_close
				arr[1]=self.hand_ang_close
			else:
				arr[0]=self.hand_ang_open
				arr[1]=self.hand_ang_open
			#shooter
			for i in range(4):
				arr[i+2]=self.shooter_state[i]
			#ball
			if self.is_shooted and self.is_ball_flying:
				count = time.time() - self.shoot_timestamp
				for i in range(3):
					self.ball_state[i+3] = self.ball_initial_state[i+3]
					self.ball_state[i] = self.ball_initial_state[i]
					self.ball_state[i] += count * self.ball_initial_state[i+3]
				self.ball_state[4] -= self.GRAVITY * count
				self.ball_state[1] -= (self.GRAVITY/2.0)*(count**2)
			elif not self.is_shooted:
				self.ball_state[0]=self.shooter_state[0]
				self.ball_state[1]=0.0
				self.ball_state[2]=self.shooter_state[1]
			for i in range(3):
				arr[i+6]=self.ball_state[i]
			#publish
			j_s.position = arr
			try:
				self.pub_viz.publish(j_s)
			except rospy.ROSException: pass


def main():
	rospy.init_node('visualizer', anonymous=True)
	node = VisualizerNode()
	rospy.spin()


if __name__ == '__main__':
	main()

