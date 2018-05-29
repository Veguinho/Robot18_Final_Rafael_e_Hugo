# # import the necessary packages
# import numpy as np
# import imutils
# import cv2
# import rospy

# class Stitcher:
# 	def __init__(self):
# 		# determine if we are using OpenCV v3.X
# 		self.isv3 = imutils.is_cv3()

# 	def stitch(self, images, ratio=0.75, reprojThresh=4.0):
# 		# unpack the images, then detect keypoints and extract
# 		# local invariant descriptors from them
# 		(imageB, imageA) = images
# 		(kpsA, featuresA) = self.detectAndDescribe(imageA)
# 		(kpsB, featuresB) = self.detectAndDescribe(imageB)

# 		#kp = []

# 		#for point in kpsA:
# 		#	temp = cv2.KeyPoint(x=point[0][0],y=point[0][1],_size=point[1], _angle=point[2], _response=point[3], _octave=point[4], _class_id=point[5]) 
# 		#	kp.append(temp)

# 		#print(kp)

# 		#result1 = cv2.drawKeypoints(imageA, kp, outImage=np.array([]))
# 		#cv2.imwrite("Result", result1)

# 		# match features between the two images
# 		M = self.matchKeypoints(kpsA, kpsB,
# 			featuresA, featuresB, ratio, reprojThresh)

# 		# if the match is None, then there aren't enough matched
# 		# keypoints to create a panorama
# 		if M is None:
# 			return None

# 		# otherwise, apply a perspective warp to stitch the images
# 		# together
# 		(matches, H, status) = M
# 		result = cv2.warpPerspective(imageA, H, (imageB.shape[1] + imageA.shape[1], imageA.shape[0]))
# 		result[0:imageB.shape[0], 0:imageB.shape[1]] = imageB

# 		# return the stitched image
# 		return result

# 	def detectAndDescribe(self, image):
# 		# convert the image to grayscale
# 		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# 		# check to see if we are using OpenCV 3.X
# 		if self.isv3:
# 			# detect and extract features from the image
# 			descriptor = cv2.xfeatures2d.SIFT_create()
# 			(kps, features) = descriptor.detectAndCompute(image, None)

# 		# otherwise, we are using OpenCV 2.4.X
# 		else:
# 			# detect keypoints in the image
# 			detector = cv2.FeatureDetector_create("SIFT")
# 			kps = detector.detect(gray)

# 			# extract features from the image
# 			extractor = cv2.DescriptorExtractor_create("SIFT")
# 			(kps, features) = extractor.compute(gray, kps)

# 		# convert the keypoints from KeyPoint objects to NumPy
# 		# arrays
# 		#resultado1 = cv2.drawKeypoints(image, kps, outImage=np.array([]))
# 		#cv2.imwrite("Result", resultado1)

# 		kps = np.float32([kp.pt for kp in kps])



# 		# return a tuple of keypoints and features
# 		return (kps, features)

# 	def matchKeypoints(self, kpsA, kpsB, featuresA, featuresB,
# 		ratio, reprojThresh):
# 		# compute the raw matches and initialize the list of actual
# 		# matches
# 		matcher = cv2.DescriptorMatcher_create("BruteForce")
# 		rawMatches = matcher.knnMatch(featuresA, featuresB, 2)
# 		matches = []

# 		# loop over the raw matches
# 		for m in rawMatches:
# 			# ensure the distance is within a certain ratio of each
# 			# other (i.e. Lowe's ratio test)
# 			if len(m) == 2 and m[0].distance < m[1].distance * ratio:
# 				matches.append((m[0].trainIdx, m[0].queryIdx))

# 		# computing a homography requires at least 4 matches
# 		if len(matches) > 4:
# 			# construct the two sets of points
# 			ptsA = np.float32([kpsA[i] for (_, i) in matches])
# 			ptsB = np.float32([kpsB[i] for (i, _) in matches])

# 			# compute the homography between the two sets of points
# 			(H, status) = cv2.findHomography(ptsA, ptsB, cv2.RANSAC,
# 				reprojThresh)

# 			# return the matches along with the homograpy matrix
# 			# and status of each matched point
# 			return (matches, H, status)

# 		# otherwise, no homograpy could be computed
# 		return None

# import the necessary packages
# import numpy as np
# import imutils
# import cv2

# class Stitcher:
# 	def __init__(self):
# 		# determine if we are using OpenCV v3.X
# 		self.isv3 = imutils.is_cv3()

# 	def stitch(self, images, ratio=10.75, reprojThresh=4.0):
# 		# unpack the images, then detect keypoints and extract
# 		# local invariant descriptors from them
# 		(imageB, imageA) = images
# 		(kpsA, featuresA) = self.detectAndDescribe(imageA)
# 		(kpsB, featuresB) = self.detectAndDescribe(imageB)

# 		# match features between the two images
# 		M = self.matchKeypoints(kpsA, kpsB,
# 			featuresA, featuresB, ratio, reprojThresh)

# 		# if the match is None, then there aren't enough matched
# 		# keypoints to create a panorama
# 		if M is None:
# 			return None

# 		# otherwise, apply a perspective warp to stitch the images
# 		# together
# 		(matches, H, status) = M
# 		#result = cv2.warpPerspective(imageA, H, (imageA.shape[1] + imageB.shape[1], imageA.shape[0] + imageB.shape[0])) # somar altura
# 		# TRocar isto
# 		#result[imageB.shape[0]:result.shape[0], imageB.shape[1]:result.shape[1]] = imageB
# 		# Add weighted
# 		#result = cv2.addWeighted(imageA, 0.5, result, 0.5, 0)
# 		#result = [0:imageB.shape[0], 0:imageB.shape[1]]


# 		heightA, widthA = imageA.shape[0], imageA.shape[1]
# 		heightB, widthB = imageB.shape[0], imageB.shape[1]
# 		#create blank image that will be large enough to hold stitched image
# 		blank_image = np.zeros(((widthA + widthB),(heightA + heightB),3),np.uint8)
# 		#stitch image two into the resulting image while using blank_image 
# 		#to create a large enough frame for images
# 		result = cv2.warpPerspective((imageA),H,blank_image.shape[0:2])
# 		#numpy notation for slicing a matrix together allows you to see the image
# 		result[0:imageB.shape[0], 0:imageB.shape[1]] = imageB
# 		# return the stitched image
# 		return result

# 	def detectAndDescribe(self, image):
# 		# convert the image to grayscale
# 		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# 		# check to see if we are using OpenCV 3.X
# 		if self.isv3:
# 			# detect and extract features from the image
# 			descriptor = cv2.xfeatures2d.SIFT_create()
# 			(kps, features) = descriptor.detectAndCompute(image, None)

# 		# otherwise, we are using OpenCV 2.4.X
# 		else:
# 			# detect keypoints in the image
# 			detector = cv2.FeatureDetector_create("SIFT")
# 			kps = detector.detect(gray)

# 			# extract features from the image
# 			extractor = cv2.DescriptorExtractor_create("SIFT")
# 			(kps, features) = extractor.compute(gray, kps)

# 		# convert the keypoints from KeyPoint objects to NumPy
# 		# arrays
# 		kps = np.float32([kp.pt for kp in kps])

# 		# return a tuple of keypoints and features
# 		return (kps, features)

# 	def matchKeypoints(self, kpsA, kpsB, featuresA, featuresB,
# 		ratio, reprojThresh):
# 		# compute the raw matches and initialize the list of actual
# 		# matches
# 		matcher = cv2.DescriptorMatcher_create("BruteForce")
# 		rawMatches = matcher.knnMatch(featuresA, featuresB, 2)
# 		matches = []

# 		# loop over the raw matches
# 		for m in rawMatches:
# 			# ensure the distance is within a certain ratio of each
# 			# other (i.e. Lowe's ratio test)
# 			if len(m) == 2 and m[0].distance < m[1].distance * ratio:
# 				matches.append((m[0].trainIdx, m[0].queryIdx))

# 		# computing a homography requires at least 4 matches
# 		if len(matches) > 4:
# 			# construct the two sets of points
# 			ptsA = np.float32([kpsA[i] for (_, i) in matches])
# 			ptsB = np.float32([kpsB[i] for (i, _) in matches])
# 			# compute the homography between the two sets of points
# 			(H, status) = cv2.findHomography(ptsA, ptsB, cv2.RANSAC,
# 				reprojThresh)

# 			# return the matches along with the homograpy matrix
# 			# and status of each matched point
# 			return (matches, H, status)

# 		# otherwise, no homograpy could be computed
# 		return None
# import the necessary packages
import numpy as np
import imutils
import cv2

class Stitcher:
	def __init__(self):
		# determine if we are using OpenCV v3.X
		self.isv3 = imutils.is_cv3()

	def stitch(self, images, ratio=10.75, reprojThresh=4.0):
		# unpack the images, then detect keypoints and extract
		# local invariant descriptors from them
		(imageB, imageA) = images
		(kpsA, featuresA) = self.detectAndDescribe(imageA)
		(kpsB, featuresB) = self.detectAndDescribe(imageB)

		# match features between the two images
		M = self.matchKeypoints(kpsA, kpsB,
			featuresA, featuresB, ratio, reprojThresh)

		# if the match is None, then there aren't enough matched
		# keypoints to create a panorama
		if M is None:
			return None

		# otherwise, apply a perspective warp to stitch the images
		# together
		(matches, H, status) = M
		result = cv2.warpPerspective(imageA, H, (imageA.shape[1] + imageB.shape[1], imageA.shape[0])) # somar altura
		# TRocar isto
		#result[0:imageB.shape[0], 0:imageB.shape[1]] = imageB
		# Add weighted
		result = cv2.addWeighted(imageA, 0.5, result, 0.5, 0)


		# return the stitched image
		return result

	def detectAndDescribe(self, image):
		# convert the image to grayscale
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		# check to see if we are using OpenCV 3.X
		if self.isv3:
			# detect and extract features from the image
			descriptor = cv2.xfeatures2d.SIFT_create()
			(kps, features) = descriptor.detectAndCompute(image, None)

		# otherwise, we are using OpenCV 2.4.X
		else:
			# detect keypoints in the image
			detector = cv2.FeatureDetector_create("SIFT")
			kps = detector.detect(gray)

			# extract features from the image
			extractor = cv2.DescriptorExtractor_create("SIFT")
			(kps, features) = extractor.compute(gray, kps)

		# convert the keypoints from KeyPoint objects to NumPy
		# arrays
		kps = np.float32([kp.pt for kp in kps])

		# return a tuple of keypoints and features
		return (kps, features)

	def matchKeypoints(self, kpsA, kpsB, featuresA, featuresB,
		ratio, reprojThresh):
		# compute the raw matches and initialize the list of actual
		# matches
		matcher = cv2.DescriptorMatcher_create("BruteForce")
		rawMatches = matcher.knnMatch(featuresA, featuresB, 2)
		matches = []

		# loop over the raw matches
		for m in rawMatches:
			# ensure the distance is within a certain ratio of each
			# other (i.e. Lowe's ratio test)
			if len(m) == 2 and m[0].distance < m[1].distance * ratio:
				matches.append((m[0].trainIdx, m[0].queryIdx))

		# computing a homography requires at least 4 matches
		if len(matches) > 4:
			# construct the two sets of points
			ptsA = np.float32([kpsA[i] for (_, i) in matches])
			ptsB = np.float32([kpsB[i] for (i, _) in matches])
			# compute the homography between the two sets of points
			(H, status) = cv2.findHomography(ptsA, ptsB, cv2.RANSAC,
				reprojThresh)

			# return the matches along with the homograpy matrix
			# and status of each matched point
			return (matches, H, status)

		# otherwise, no homograpy could be computed
		return None