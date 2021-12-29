#!/usr/bin/env python3

import time
import math
import rospy
from std_msgs.msg import Float32MultiArray, Bool, Float32
from sensor_msgs.msg import Joy
from robot3_18c1054.srv import GetInitTime, GetInitTimeResponse

class ShooterControllerNode():
	DEFAULT_BALL_INITIAL_SPEED = 5.0

	init_time= 0.0

	upper_limit = [10.0, 0.0, 2.0, 1.55]
	lower_limit = [-10.0, -10.0, -2.0, 0.0]
	state = [0.0, -1.0, 0.0, 0.0]

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
			print('service err in shooter_controller')
			self.init_time = time.time()

		self.pub_state = rospy.Publisher('shooter_state', Float32MultiArray, queue_size=10)
		self.pub_shoot = rospy.Publisher('ball_initial_state', Float32MultiArray, queue_size=10)
		self.sub_shoot_b = rospy.Subscriber('shoot_bool', Bool, self.shoot_bool)
		self.sub_shoot_f = rospy.Subscriber('shoot_value', Float32, self.shoot_value)
		self.sub_state = rospy.Subscriber('shooter_state_input', Float32MultiArray, self.state_reinit)


	def state_reinit(self, msg):
		if len(msg.data) >= 4:
			for i in range(4):
				self.state[i] = msg.data[i]
				if self.state[i] > self.upper_limit[i]:
					self.state[i] = self.upper_limit[i]
				elif self.state[i] < self.lower_limit[i]:
					self.state[i] = self.lower_limit[i]
			arr_pub = Float32MultiArray(data=self.state)
			self.pub_state.publish(arr_pub)


	def shoot_bool(self, msg):
		if msg.data==True:
			self.shoot(self.DEFAULT_BALL_INITIAL_SPEED)


	def shoot_value(self, msg):
		self.shoot(msg.data)


	def shoot(self, ball_initial_speed):
		if not rospy.is_shutdown():
			#calc ball's initial state(position & velocity)
			ball_initial_state = [0.0]*7
			ball_initial_state[6]=time.time() - self.init_time
			ball_initial_state[0]=self.state[0]
			ball_initial_state[1]=0.0
			ball_initial_state[2]=self.state[1]
			ball_initial_state[3]=math.sin(self.state[2])*math.cos(self.state[3])
			ball_initial_state[4]=math.sin(self.state[3])
			ball_initial_state[5]=math.cos(self.state[2])*math.cos(self.state[3])
			for i in range(3):
				ball_initial_state[i+3]*=ball_initial_speed
			#publish
			arr_pub = Float32MultiArray(data=ball_initial_state)
			self.pub_shoot.publish(arr_pub)



def main():
	rospy.init_node('shooter_controller', anonymous=True)
	node = ShooterControllerNode()
	rospy.spin()


if __name__ == '__main__':
	main()

