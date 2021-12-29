#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import Float32MultiArray

spring_const = 1
ball_mass = 1
gravity = -9.8

def main():
	listener()


def talker(spx, spz, ssa, sr, ss):
	pub = rospy.Publisher('ball_highest_pos', Float32MultiArray, queue_size = 10)
	#calc ball's initial velocity
	ball_init_v = [0, 0, 0]
	ball_init_v[0] = math.sin(sr) * math.cos(ssa)
	ball_init_v[1] = math.sin(ssa)
	ball_init_v[2] = math.cos(sr) * math.cos(ssa)
	global spring_const
	global ball_mass
	init_speed = ss
	init_speed *= math.sqrt(spring_const / ball_mass)
	for i in range(3):
		ball_init_v[i] *= init_speed

	#calc ball's state(time & position & velocity) when it's highest position
	global gravity
	time_till_ball_highest = - ball_init_v[1] / gravity
	ball_highest_pos = [0, 0, 0]
	ball_highest_pos[0] = spx + ball_init_v[0] * time_till_ball_highest
	ball_highest_pos[1] = - (0.5 * (ball_init_v[1] ** 2)) / gravity
	ball_highest_pos[2] = spz + ball_init_v[2] * time_till_ball_highest
	ball_highest_v = [0, 0, 0]
	ball_highest_v[0] = ball_init_v[0]
	ball_highest_v[1] = 0
	ball_highest_v[2] = ball_init_v[2]

	arr = []
	arr.append(time_till_ball_highest)
	for i in range(3):
		arr.append(ball_highest_pos[i])
	for i in range(3):
		arr.append(ball_highest_v[i])
	arr_pub = Float32MultiArray(data=arr)
	if not rospy.is_shutdown():
		pub.publish(arr_pub)


def callback(data):
	shooter_pos_x = data.data[0]
	shooter_pos_z = data.data[1]
	shooter_slope_ang = data.data[2]
	shooter_roll = data.data[3]
	shooter_spring = data.data[4]

	try:
		talker(shooter_pos_x, shooter_pos_z, shooter_slope_ang, shooter_roll, shooter_spring)
	except rospy.ROSInterruptException:
		print('err in shooter_posture_to_ball_highest_pos')


def listener():
	rospy.init_node('shooter_posture_to_ball_highest_pos', anonymous = True)
	rospy.Subscriber('shooter_posture', Float32MultiArray, callback)
	rospy.spin()


if __name__ == '__main__':
	main()
