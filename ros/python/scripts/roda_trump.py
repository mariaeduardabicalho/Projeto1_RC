#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import numpy as np
import time
import rospy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import math

bridge = CvBridge()

trump=cv2.imread('/home/borg/catkin_ws/src/robot18/ros/exemplos_python/scripts/trump.jpeg')
atraso = 1.5
check_delay = False
media = []
centro = []

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
	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.secs

	if delay > atraso and check_delay==True:
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro = image_detector(trump, cv_image)
		cv2.imshow("Camera", trump)
	except CvBridgeError as e:
		print('ex', e)
  

if __name__=="__main__":

    rospy.init_node("cor")
    # Para usar a Raspberry Pi
    recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
    
    # Para usar a webcam 
    #recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    try:

        while not rospy.is_shutdown():
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            if len(media) != 0 and len(centro) != 0:
                dif_x = media[0]-centro[0]
                dif_y = media[1]-centro[1]
                if math.fabs(dif_x)<30: # Se a media estiver muito proxima do centro anda para frente
                    vel = Twist(Vector3(0.5,0,0), Vector3(0,0,0))
                else:
                    if dif_x > 0: # Vira a direita
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
                    else: # Vira a esquerda
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,0.2))
            velocidade_saida.publish(vel)
            rospy.sleep(0.01)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")