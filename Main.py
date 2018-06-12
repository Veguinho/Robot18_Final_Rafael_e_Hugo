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
import argparse
import imutils
from std_msgs.msg import Empty
import keyboard
from pano import Stitch

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
counter2 = 0

maxImagens = 4

listaFotos = []

estado = "landed"

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
	global counter2

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.nsecs

	if counter2%50 != 0:
		counter2 += 1

		return
	try:
		counter2 += 1
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		#scaneou(cv_image)
		depois = time.clock()
		if counter < maxImagens and timerW == 1:
			#rospy.sleep(dormir)
			cv2.imwrite("images/Camera_"+ str(counter) + ".png", cv_image)
			counter += 1
			print(counter)
			listaFotos.append(cv_image)


	except CvBridgeError as e:
		print('ex', e)



timerW = 0
# main
def main():
	global velocidade_saida
	global buffer
	global menorDist
	global aceleracao
	global counter
	global listaFotos
	global estado
	global timerW

	rospy.init_node('cor_estados')

	# Para usar a webcam
	#recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
	recebedor = rospy.Subscriber("/bebop/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)
	takeoff_publisher = rospy.Publisher("bebop/takeoff", Empty, queue_size = 1)
	land_publisher = rospy.Publisher("bebop/land", Empty, queue_size = 1)
	velocidade_saida = rospy.Publisher("bebop/cmd_vel", Twist, queue_size = 1)

	timerW = 0



	while not rospy.is_shutdown():
		if estado == "landed":
			rospy.sleep(0.5)
			takeoff_publisher.publish(Empty())
			estado = "voando"
			print(estado)
			rospy.sleep(2.3)
			timerW = 1

		vel = Twist(Vector3(0, 0.2, 0), Vector3(0, 0, 0))
		rospy.sleep(0.1)
		velocidade_saida.publish(vel)

		if counter >= maxImagens:
			rospy.sleep(0.5)
			land_publisher.publish(Empty())
			print("FIM")
			return

	





if __name__ == '__main__':
	main()
	try:
		args = sys.argv[1]
	except:
		args = "txtlists/files1.txt"
	finally:
		print "Parameters : ", args
	s = Stitch(args)
	s.leftshift()
	# s.showImage('left')
	s.rightshift()
	print "Pronto!"
	cv2.imwrite("imagem_final.jpg", s.leftImage)
	print "Imagem Final pronta!"
	cv2.destroyAllWindows()
