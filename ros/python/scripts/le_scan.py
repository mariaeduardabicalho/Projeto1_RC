#! /usr/bin/env python
# -*- coding:utf-8 -*-
import le_imu
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
import transformations
import math


parar = False
def scaneou(dado):
	# print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	# print("Leituras:")
	#r = np.array(dado.ranges).round(decimals=2)
	#maior valor dps do 0

	# print(dado.range_min)
	p=10
	for x in dado.ranges:
		if x < p and x>0 :
			perto= x
	#perto = min(dado.ranges)
	#print(perto)
	if perto < 0.4 :
		global parar
		parar = True
		#print("entrou")
		velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(velocidade)
	else:
		global parar
		parar = False
	#print("Intensities")
	#print(np.array(dado.intensities).round(decimals=2))

if __name__=="__main__":

	rospy.init_node("le_scan")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
	recebe_imu = rospy.Subscriber("/imu", Imu, le_imu.leu_imu,queue_size = 1)

	while not rospy.is_shutdown():
		print("Oeee")
		print(le_imu.bateu)
		print(parar)
		if parar == False:
			velocidade = Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(velocidade)
		if le_imu.bateu == True:
			#print(bateu)
			velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(velocidade)
		rospy.sleep(1)
