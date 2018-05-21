#! /usr/bin/env python
# -- coding:utf-8 --

_author_ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]
_coauthor_ = ['Hugo Carl', 'Rafael Viera']

import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
import math
from pyimagesearch.panorama import Stitcher
import argparse
import imutils
#import featuremodule
#import featurestest

dormir = 1

atraso = 2E9


bridge = CvBridge()

cv_image = None

# Atraso máximo permitido entre a imagem sair do Turbletbot3 e chegar no laptop do aluno
#atraso = 1.5
check_delay = False # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados

counter = 0

maxImagens = 2

listaFotos = []

def roda_todo_frame(imagem):
	global cv_image
	global media_cor
	global centro
	global area
	global menorDist
	global aceleracao
	global media_feature
	global centro_feature
	global counter 
	global listaFotos
	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.nsecs
	if delay > atraso and check_delay==True:
		return
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		#scaneou(cv_image)
		depois = time.clock()
		#cv2.imwrite("Camera_"+ str(counter) + ".png", cv_image)
		listaFotos.append(cv_image)
	except CvBridgeError as e:
		print('ex', e)




# main
def main():
	global velocidade_saida
	global buffer
	global menorDist
	global aceleracao
	global counter
	rospy.init_node('cor_estados')

	# Para usar a webcam
	#recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
	recebedor = rospy.Subscriber("/bebop/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

	
    
	while not rospy.is_shutdown():
		rospy.sleep(dormir)
		counter+= 1
		if counter >= maxImagens:
			print("FIM")
			result = listaFotos[0]
			for i in range(0, len(listaFotos)):
				imageB = listaFotos[i]
				#imageA = imutils.resize(imageA, width=500)
				#imageB = imutils.resize(imageB, width=500)
				stitcher = Stitcher()
				result = stitcher.stitch([result, imageB])
			cv2.imwrite("CameraFinal.png", result)
			print("IMAGEM CRIADA")
			return

	





if __name__ == '__main__':
	main()
