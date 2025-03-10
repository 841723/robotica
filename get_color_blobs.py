# -*- coding: utf-8 -*-
#!/usr/bin/python

# SCRIPT PARA CALIBRAR LOS VALORES DE COLOR Y TAMAÑO DE LOS BLOBS


# Standard imports
import cv2
import numpy as np;

# Read image
img_BGR = cv2.imread("./fotos/imagenes/image_1741256867.824534.jpg") # TODO: CAMBIAR A HSV
#img_BGR = cv2.imread("many.jpg")
img_flip = cv2.flip(img_BGR, 0)
img_HSV = cv2.cvtColor(img_flip, cv2.COLOR_BGR2HSV)

# Setup default values for SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()
  
# These are just examples, tune your own if needed
# Change thresholds
params.minThreshold = 10
params.maxThreshold = 200

# Filter by Area
params.filterByArea = True
params.minArea = 200
params.maxArea = 10000

# Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.1

# Filter by Color
params.filterByColor = False
# not directly color, but intensity on the channel input
#params.blobColor = 0
params.filterByConvexity = False
params.filterByInertia = False


# Create a detector with the parameters
ver = (cv2.__version__).split('.')
if int(ver[0]) < 3 :
	detector = cv2.SimpleBlobDetector(params)
else :
	detector = cv2.SimpleBlobDetector_create(params)


cv2.imshow("Original", img_HSV)
# keypoints on original image (will look for blobs in grayscale)
keypoints = detector.detect(img_HSV)
# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
# the size of the circle corresponds to the size of blob
im_with_keypoints = cv2.drawKeypoints(img_flip, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# Show blobs
cv2.imshow("Keypoints on Gray Scale", im_with_keypoints)
cv2.waitKey(0)

# filter certain COLOR channels

# Pixels with 100 <= R <= 255, 15 <= B <= 56, 17 <= G <= 50 will be considered red.
# similar for BLUE

# BY DEFAULT, opencv IMAGES have BGR format
redMin = (0, 25, 40)
redMax = (20, 50, 255)  # TODO: cuando vuelva a 360 cómo?

blueMin=(60, 10, 10)
blueMax=(255, 100, 100)

lower_red1 = (0, 100, 100)
upper_red1 = (10, 255, 255)
lower_red2 = (160, 100, 100)
upper_red2 = (179, 255, 255)

# Blue in HSV typically has hue values around 100-130
lower_blue = (100, 100, 100)
upper_blue = (130, 255, 255)

mask_red1 = cv2.inRange(img_HSV, lower_red1, upper_red1)
mask_red2 = cv2.inRange(img_HSV, lower_red2, upper_red2)
mask_red = cv2.bitwise_or(mask_red1, mask_red2)


# mask_red=cv2.inRange(img_HSV, redMin, redMax)
mask_blue=cv2.inRange(img_HSV, blueMin, blueMax)


# apply the mask
red = cv2.bitwise_and(img_HSV, img_HSV, mask = mask_red)
blue = cv2.bitwise_and(img_HSV, img_HSV, mask = mask_blue)
# show resulting filtered image next to the original one
cv2.imshow("Red regions", np.hstack([img_flip, red]))
cv2.imshow("Blue regions", np.hstack([img_flip, blue]))


# detector finds "dark" blobs by default, so invert image for results with same detector
keypoints_red = detector.detect(255-mask_red)
keypoints_blue = detector.detect(255-mask_blue)

# documentation of SimpleBlobDetector is not clear on what kp.size is exactly, but it looks like the diameter of the blob.
for kp in keypoints_red:
    print(kp.pt[0], kp.pt[1], kp.size)

# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
# the size of the circle corresponds to the size of blob
im_with_keypoints = cv2.drawKeypoints(img_flip, keypoints_red, np.array([]),
	(255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
im_with_keypoints2 = cv2.drawKeypoints(img_flip, keypoints_blue, np.array([]),
	(255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# Show mask and blobs found
cv2.imshow("Keypoints on RED", im_with_keypoints)
cv2.imshow("Keypoints on BLUE", im_with_keypoints2)
cv2.waitKey(0)

