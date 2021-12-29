#!/usr/bin/env python3

import math
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from test.srv import GetHandState, GetHandStateResponse


class BallToTargetNode():
	GRAVITY = 9.8


	def __init__(self):
		self.pub_arm = rospy.Publisher('target_arm_state', Float32MultiArray, queue_size=10)
		self.pub_hand = rospy.Publisher('hand_close_reserve', Float32, queue_size=10)
		self.sub = rospy.Subscriber('ball_initial_state', Float32MultiArray, self.update_target)


	def update_target(self, msg):
		time_till_highest = msg.data[4] / 9.8
		if time_till_highest < 0.0:
			time_till_highest = 0.0

		ball_highest_pos = [0.0]*3
		for i in range(3):
			ball_highest_pos[i] = msg.data[i] + time_till_highest*msg.data[i+3]
		ball_highest_pos[1] -= (9.8/2.0)*(time_till_highest**2)

		ball_highest_att = [[0,0,0], [0,0,0], [0,0,0]]
		ball_highest_att[1][1] = 1.0
		temp = msg.data[3]**2 + msg.data[5]**2
		temp = math.sqrt(temp)
		if not temp == 0.0:
			ball_highest_att[0][2] = msg.data[3] / temp
			ball_highest_att[2][2] = msg.data[5] / temp
		else:
			ball_highest_att[2][2] = 1.0
		ball_highest_att[0][0] = ball_highest_att[2][2]
		ball_highest_att[2][0] = -ball_highest_att[0][2]

		#pulish
		state = [0.0]*24
		for i in range(3):
			state[i] = ball_highest_pos[i]
			for j in range(3):
				state[i*3+j+6] = ball_highest_att[i][j]
		close_time = msg.data[6] #time when shoot
		close_time += time_till_highest

		time_pub = Float32(close_time)
		self.pub_hand.publish(time_pub)
		state_pub = Float32MultiArray(data=state)
		self.pub_arm.publish(state_pub)

	'''
	def update_target_2(self, msg):
		rospy.wait_for_service('get_hand_state')
		try:
			#search point in ball's trajctry which nearest from hand
			get_hand_state = rospy.ServiceProxy('get_hand_state', GetHandState)
			resp = get_hand_state()
			print('current_hand_state : ',resp.hand_state)
			rel_state = [(msg.data[i] - resp.hand_state[i]) for i in range(6)]
			print('current relative state : ',rel_state)
			poly_coefs = [0.0]*4
			poly_coefs[0] = 1.0
			poly_coefs[1] = -3.0*rel_state[4]/self.GRAVITY
			poly_coefs[2] = -rel_state[1]/(2.0*self.GRAVITY)
			for i in range(3):
				poly_coefs[2] += (rel_state[i+3]/self.GRAVITY)**2
				poly_coefs[3] += rel_state[i]*rel_state[i+3]
			poly_coefs[2] *= 4.0
			poly_coefs[3] *= 2.0
			poly_coefs[3] /= self.GRAVITY**2
			poly_coefs[3] -= 1.0
			print('oh yeah : ', poly_coefs)
			poly_roots = np.roots(poly_coefs)
			print('catch time candidate : ',poly_roots)

			#calcurate the point's time
			time_catch = 0.0
			for i in range(3):
				if poly_roots[i].imag == 0.0 and poly_roots[i].real > time_catch:
					time_catch = poly_roots[i].real

			#calcurate the point's position
			pos = [0.0]*3
			for i in range(3):
				pos[i] = msg.data[i] + time_catch*msg.data[i+3]
			pos[1] -= (self.GRAVITY/2.0)*(time_catch**2)

			#calcurate the point's attitude
			att_y = np.zeros(3)
			att_y[1] = 1.0

			att_z = np.zeros(3)
			for i in range(3):
				att_z[i] = msg.data[i+3]
			att_z[1] -= time_catch*msg.data[4]
			temp = np.linalg.norm(att_z)
			if temp == 0.0:
				att_z[0]=0.0
				att_z[1]=0.0
				att_z[2]=1.0
			else:
				att_z = att_z / temp

			att_x = np.cross(att_y,att_z)
			#att_z is never vartical, so don't need to think about att_x=0
			att_x = att_x / np.linalg.norm(att_x)

			att_y = np.cross(att_z,att_x)

			#pulish
			state = [0.0]*24
			for i in range(3):
				state[i] = pos[i]
				state[i*3 + 0 + 6] = att_x[i]
				state[i*3 + 1 + 6] = att_y[i]
				state[i*3 + 2 + 6] = att_z[i]
			close_time = msg.data[6] #time when shoot
			close_time += time_catch

			print('--target--')
			print('time : ',time_catch)
			print('pos  : ',pos)
			print('att  :')
			print(att_x)
			print(att_y)
			print(att_z)

			time_pub = Float32(close_time)
			self.pub_hand.publish(time_pub)
			state_pub = Float32MultiArray(data=state)
			self.pub_arm.publish(state_pub)
		except rospy.ServiceException:
			update_target_1(msg)
	'''

def main():
	rospy.init_node('ball_initial_to_arm_target', anonymous=True)
	node = BallToTargetNode()
	rospy.spin()


if __name__ == '__main__':
	main()
