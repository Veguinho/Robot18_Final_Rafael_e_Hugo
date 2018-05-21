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
#import featuremodule
#import featurestest

dormir = 1.5

atraso = 2E9


bridge = CvBridge()

cv_image = None

# Atraso máximo permitido entre a imagem sair do Turbletbot3 e chegar no laptop do aluno
#atraso = 1.5
check_delay = False # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados

counter = 0

def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media_cor
	global centro
	global area
	global menorDist
	global aceleracao
	global media_feature
	global centro_feature
	global counter 
	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.nsecs
	print("INICIO")
	if delay > atraso and check_delay==True:
		return
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		#scaneou(cv_image)
		depois = time.clock()
		cv2.imwrite("Camera_"+ str(counter) + ".png", cv_image)
		print("IMAGEM CRIADA")
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


	# Create a SMACH state machine
	#sm = smach.StateMachine(outcomes=['terminei'])

	# Open the container
	# with sm:
	# 	# Add states to the container
	# 	#smach.StateMachine.add('LONGE', Longe(),
	# 	#                       transitions={'ainda_longe':'ANDANDO',
	# 	#                                    'perto':'terminei'})
	# 	#smach.StateMachine.add('ANDANDO', Andando(),
	# 	#                       transitions={'ainda_longe':'LONGE'})
	# 	smach.StateMachine.add('GIRANDO', Girando(),
	# 							transitions={'brecar':'PARAR',
	# 							'ré': 'FUGIR',
	# 							'girando': 'GIRANDO',
	# 							'alinhou':'CENTRO'})
	# 	smach.StateMachine.add('CENTRO', Centralizado(),
	# 							transitions={'alinhando': 'GIRANDO',
	# 							'alinhado':'CENTRO',
	# 							'brecar':'PARAR'})
	# 	smach.StateMachine.add('PARAR', Parar(),
	# 							transitions={'girando': 'GIRANDO',
	# 							'brecar':'PARAR'})
	# 	smach.StateMachine.add('FUGIR', Fugir(),
	# 							transitions={'girando': 'GIRANDO',
	# 							'brecar':'PARAR',
	# 							'ré' : 'FUGIR'})

	# Execute SMACH plan
	#outcome = sm.execute()
	#rospy.spin()


if __name__ == '__main__':
	main()
