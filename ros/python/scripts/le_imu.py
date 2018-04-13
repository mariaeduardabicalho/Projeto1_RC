#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu
import transformations
import math

bateu = False

# ax=[0.2]
# ay=[0.05]
t = 5
ax = np.zeros(t, dtype=float)
ay = np.zeros(t, dtype=float)
i = 0

def leu_imu(dado):
	global i
	#velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
	#velocidade_saida.p
	quat = dado.orientation
	# velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
	#velocidade_saida.p
	lista = [quat.x, quat.y, quat.z, quat.w]
	angulos = np.degrees(transformations.euler_from_quaternion(lista))
	mensagem = """
	Tempo: {:}
	Orientação: {:.2f}, {:.2f}, {:.2f}
	Vel. angular: x {:.2f}, y {:.2f}, z {:.2f}\
	Aceleração linear:
	x: {:.2f}
	y: {:.2f}
	z: {:.2f}
""".format(dado.header.stamp, angulos[0], angulos[1], angulos[2], dado.angular_velocity.x, dado.angular_velocity.y, dado.angular_velocity.z, dado.linear_acceleration.x, dado.linear_acceleration.y, dado.linear_acceleration.z)
	#print(mensagem)

	global ax
	ax[i%t]=dado.linear_acceleration.x

	global ay
	ay[i%t]=dado.linear_acceleration.y
	i+=1
	#achar media
	mediax= np.mean(ax)
	#print(mediax)
	dif_ax= math.sqrt((ax[-1] - mediax)**2)
	bateu = False
	if dif_ax > 2:
		global bateu
		bateu = True
		# print(dif_ax)

	mediay= np.mean(ay)
	dif_ay= math.sqrt((ay[-1] - mediay)**2)

	if dif_ay > 1:
		global bateu
		bateu = True
		# print("y : ",dif_ay)

	# print(bateu)

if __name__=="__main__":

	rospy.init_node("le_imu")

	recebe_scan = rospy.Subscriber("/imu", Imu, leu_imu)

	while not rospy.is_shutdown():
		print("Main loop")


		print("\tBATEU ", bateu)
		rospy.sleep(2)
