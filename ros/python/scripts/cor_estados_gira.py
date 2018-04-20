#! /usr/bin/env python
# -- coding:utf-8 --

#_author_ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import cv2
import numpy as np
import time
import rospy
import tf
import math
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
import le_scan
import le_imu
import cormodule


bridge = CvBridge()
cv_image = None
trump=cv2.imread('/home/dduda/catkin_ws/src/robot18/ros/exemplos_python/scripts/trump.jpeg')

# Variáveis para permitir que o roda_todo_frame troque dados com a máquina de estados
media = []
centro = []
area = 0.0
media2=[]
centro2=[]
tolerancia_x = 50
tolerancia_y = 20
ang_speed = 0.4
area_ideal = 60000 # área da distancia ideal do contorno - note que varia com a resolução da câmera
tolerancia_area = 40000
# Atraso máximo permitido entre a imagem sair do Turbletbot3 e chegar no laptop do aluno
atraso = 1.5
check_delay = True # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados

limite = 10000
ang_speed = 0.7


def auto_canny(image, sigma=0.33):
	# compute the median of the single channel pixel intensities
	v = np.median(image)

	# apply automatic Canny edge detection using the computed median
	lower = int(max(0, (1.0 - sigma) * v))
	upper = int(min(255, (1.0 + sigma) * v))
	edged = cv2.Canny(image, lower, upper)

	# return the edged image
	return edged


def image_detector(img, frame):
	mean = []
	minLines=15
	sift = cv2.xfeatures2d.SIFT_create()
	keyPoint1, descrpt1 = sift.detectAndCompute(img,None)
	keyPoint2, descrpt2 = sift.detectAndCompute(frame,None)

	FLANN_INDEX_KDTREE = 0
	index_params = dict(algorithm = 0, trees = 5)
	search_params = dict(checks = 50)

	flann = cv2.FlannBasedMatcher(index_params, search_params)

	lines = flann.knnMatch(descrpt1,descrpt2,k=2)

	center = (frame.shape[0]//2, frame.shape[1]//2)
	print(center , mean)
	good = []
	for m,n in lines:
		if m.distance < 0.5*n.distance:
			good.append(m)

	if len(good)>minLines:
		#Transformação para float 32
		src_points = np.float32([ keyPoint1[m.queryIdx].pt for m in good]).reshape(-1,1,2)
		dst_points = np.float32([ keyPoint2[m.trainIdx].pt for m in good]).reshape(-1,1,2)

		M, mask = cv2.findHomography(src_points, dst_points, cv2.RANSAC, 5.0)
		matchesMask = mask.ravel().tolist()
		h,w = img.shape[0], img.shape[1]
		pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)

		dst = cv2.perspectiveTransform(pts,M)

		for i in range(len(dst)):
			x=0
			y=0
			mean = []
			x += dst[i][0][0]
			y += dst[i][0][1]
			xmed=x/len(dst)
			ymed=y/len(dst)
			mean.append(xmed)
			mean.append(ymed)


		frame = cv2.polylines(frame,[np.int32(dst)],True,255,3,cv2.LINE_AA)
		print("Achou")

	else:
		print("Não achou")
		matchesMask = None

	draw_params = dict(matchColor = (0,255,0),singlePointColor = None,matchesMask = matchesMask,flags=2)

	return mean, center




def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro
	global area
	global media2
	global centro2

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.secs
	if delay > atraso and check_delay==True:
		return
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro, area = cormodule.identifica_cor(cv_image)
		media2, centro2 = image_detector(trump,cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)






## Classes - estados


class Girando(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['alinhou', 'girando', 'alinhou2'])

	def execute(self, userdata):
		global velocidade_saida

		if le_scan.parar == True or le_imu.bateu == True:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			rospy.sleep(1)
			return 'girando'

		elif media is not None and len(media)!=0 and centro is not None and len(centro)!=0 :
			
			if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.5))
				velocidade_saida.publish(vel)
				return 'girando'

			if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.5))
				velocidade_saida.publish(vel)
				return 'girando'

			else:
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				return 'alinhou'

		elif media2 is not None and len(media2)!=0 and centro2 is not None and len(centro2)!=0:
			
			if math.fabs(media2[0]-centro2[0]) > 0:
				vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.5))
				return 'girando'

			if math.fabs(media2[0]-centro2[0]) < 0:
				vel = Twist(Vector3(0,0,0), Vector3(0,0,0.))
				velocidade_saida.publish(vel)
				return 'girando'

			else:
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				return 'alinhou2'

		else:
			return 'girando'


class Centralizado(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['alinhando', 'alinhado'])

	def execute(self, userdata):
		global velocidade_saida
	
		if le_scan.parar == True or le_imu.bateu == True:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			rospy.sleep(1)
			return 'girando'

		if media is None:
			return 'alinhou'
		if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
			return 'alinhando'
		if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
			return 'alinhando'
		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'alinhado'


class Gira180(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['fugir','girando'])
		self.numero_voltas = 0
	def execute(self, userdata):
		if le_scan.parar == True or le_imu.bateu == True:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			rospy.sleep(1)
			return 'girando'
		self.numero_voltas +=1
		if self.numero_voltas < limite:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed))
			velocidade_saida.publish(vel)
			return 'girando'
		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			self.numero_voltas = 0
			return 'fugir'


class Centralizado2(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['alinhando2', 'alinhado2'])

	def execute(self, userdata):
		global velocidade_saida

		if le_scan.parar == True or le_imu.bateu == True:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			rospy.sleep(1)
			return 'girando'

		if media is None:
			return 'alinhou2'
		if  math.fabs(media2[0]) > math.fabs(centro2[0] + tolerancia_x):
			return 'alinhando2'
		if math.fabs(media2[0]) < math.fabs(centro2[0] - tolerancia_x):
			return 'alinhando2'
		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'alinhado2'

class Segue(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['fim'])
	def execute(self, userdata):
		if le_scan.parar == True or le_imu.bateu == True:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			rospy.sleep(1)
			return 'fim'
		else:
			vel = Twist(Vector3(0.3, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'fim'

class Fugir(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['fugir', 'fim'])
		self.numero_voltas = 0
	def execute(self, userdata):
		self.numero_voltas +=1
		if le_scan.parar == True or le_imu.bateu == True:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			rospy.sleep(1)
			return 'fim'
		if self.numero_voltas < limite:
			vel = Twist(Vector3(0.3, 0, 0), Vector3(0, 0,0))
			velocidade_saida.publish(vel)
			return 'fugir'
		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			self.numero_voltas = 0
			return 'fim'

# main
def main():
	global velocidade_saida
	global buffer
	rospy.init_node('cor_estados')

	# Para usar a webcam
	#recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
	recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
	recebe_scan = rospy.Subscriber("/scan",LaserScan, le_scan.scaneou)
	recebe_imu = rospy.Subscriber("/imu", Imu, le_imu.leu_imu,queue_size = 1)

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['terminei'])

	# Open the container
	with sm:
		# Add states to the container
		#smach.StateMachine.add('LONGE', Longe(),
		#                       transitions={'ainda_longe':'ANDANDO',
		#                                    'perto':'terminei'})
		#smach.StateMachine.add('ANDANDO', Andando(),
		#                       transitions={'ainda_longe':'LONGE'})
		smach.StateMachine.add('GIRANDO', Girando(),
								transitions={'girando': 'GIRANDO',
								'alinhou':'CENTRO', 'alinhou2' : 'CENTRO2'})
		smach.StateMachine.add('CENTRO', Centralizado(),
								transitions={'alinhando': 'GIRANDO',
								'alinhado':'GIRA180'})
		smach.StateMachine.add('CENTRO2', Centralizado2(),
								transitions={'alinhando2': 'GIRANDO',
								'alinhado2':'SEGUE'})
		smach.StateMachine.add('GIRA180', Gira180(),
								transitions={'fugir': 'FUGINDO',
								'girando':'GIRA180'})
		smach.StateMachine.add('SEGUE', Segue(),
								transitions={'fim': 'GIRANDO',
								})
		smach.StateMachine.add('FUGINDO', Fugir(),
								transitions={'fugir': 'FUGINDO', 'fim':'GIRANDO'})



	# Execute SMACH plan
	outcome = sm.execute()
	#rospy.spin()


if __name__ == '__main__':
	main()