# USAGE
# python stitch.py --first images/bryce_left_01.png --second images/bryce_right_01.png 

#import the necessary packages
from pyimagesearch.panorama import Stitcher
import numpy as np
import argparse
import imutils
import cv2

# # construct the argument parse and parse the arguments
# ap = argparse.ArgumentParser()
# ap.add_argument("-f", "--first", required=True,
# 	help="path to the first image")
# ap.add_argument("-s", "--second", required=True,
# 	help="path to the second image")
# args = vars(ap.parse_args())

# # load the two images and resize them to have a width of 400 pixels
# # (for faster processing)
# imageA = cv2.imread(args["first"])
# imageB = cv2.imread(args["second"])
# imageA = imutils.resize(imageA, width=1000)
# imageB = imutils.resize(imageB, width=1000)

# # stitch the images together to create a panorama
# stitcher = Stitcher()
# result = stitcher.stitch([imageA, imageB]

multiplicador = 3

result = cv2.imread("Camera_0.png")
#result = cv2.cvtColor(result, cv2.COLOR_BGR2RGB)

#fundo = np.zeros((multiplicador*result.shape[0],multiplicador*result.shape[1],3), dtype=np.uint8)

#cv2.imwrite("fundo.png", fundo)

#o = result.shape

#fundo[o[0]:(o[0]+o[0]), o[1]:(o[1]+o[1]),0:3] = result[:,:,0:3]

#cv2.imwrite("fundocopiado.png", fundo) 
#result = imutils.resize(result, width=1000)
#result = fundo
for i in range(1,3):
	imageA = result
	imageB = cv2.imread("Camera_" + str(i) + ".png")
	#imageB = listaFotos[i]
	#imageA = imutils.resize(imageA)
	#imageB = imutils.resize(imageB)
	stitcher = Stitcher()
	result = stitcher.stitch([imageA, imageB])
	print("Stich de imagem_", i-1, "com imagem_", i)
	cv2.imwrite("Camera_RESULT.png", result)
print("IMAGEM CRIADA")
