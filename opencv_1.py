# -*- coding: utf-8 -*-
"""
Created on Thu Jan 10 15:50:43 2019

@author: abhinav.jhanwar
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt
import imutils

################ create image with black background
# color, data type
pic = np.zeros((500, 500, 3), dtype='uint8')

################### load image
# load the input image (whose path was supplied via command line
# argument) and display the image to our screen
pic = cv2.imread('Abhinav.jpg')

################ show image
# display the image to our screen -- we will need to click the window
# open by OpenCV and press a key on our keyboard to continue execution
cv2.imshow("Image", pic)
cv2.waitKey(0)

####################### basics
# load the input image and show its dimensions, keeping in mind that
# images are represented as a multi-dimensional NumPy array with
# shape no. rows (height) x no. columns (width) x no. channels (depth)
(h, w, d) = pic.shape
print("width={}, height={}, depth={}".format(w, h, d))

# access the RGB pixel located at x=50, y=100, keepind in mind that
# OpenCV stores images in BGR order rather than RGB
(B, G, R) = pic[100, 50]
print("R={}, G={}, B={}".format(R, G, B))

# extract a 100x100 pixel square ROI (Region of Interest) from the
# input image starting at x=320,y=60 at ending at x=420,y=160
roi = pic[60:160, 320:420]
cv2.imshow("roi", roi)
cv2.waitKey(0)

######################### histogram
pic = cv2.imread('Abhinav.jpg')
b = [0]
g = [1]
r = [2]
# images, channel, mask: roi else None, histSize: bin count, ranges
histogram = cv2.calcHist([pic], b, None, [256], [0, 256])
# plot histogram
plt.plot(histogram, color='b');

# images, channel, mask: roi else None, histSize: bin count, ranges
histogram = cv2.calcHist([pic], g, None, [256], [0, 256])
# plot histogram
plt.plot(histogram, color='g');

# images, channel, mask: roi else None, histSize: bin count, ranges
histogram = cv2.calcHist([pic], r, None, [256], [0, 256])
# plot histogram
plt.plot(histogram, color='r');
plt.show()

# combined histogram
plt.hist(image.ravel(), 256, [0, 256]);
plt.show()

################## draw rectangle
# image, initial pos, final pos, color, width
cv2.rectangle(pic, (0,0), (500, 150), (255, 0, 0), 3, lineType=8, shift=0)

# draw a 2px thick red rectangle
image = cv2.imread("Abhinav.jpg")
output = image.copy()
cv2.rectangle(output, (320, 60), (420, 160), (0, 0, 255), 2)
cv2.imshow("Image.jpg", output)
cv2.waitKey(0)

################ draw line
# image, initial pos, final pos, color
cv2.line(pic, (0,0), (500, 150), (255, 0, 0), 5)
cv2.imshow("Image", pic)
cv2.waitKey(0)

# draw a 5px thick red line from x=60,y=20 to x=400,y=200
output = image.copy()
cv2.line(output, (60, 20), (400, 200), (0, 0, 255), 5)
cv2.imshow("Image.jpg", output)
cv2.waitKey(0)

###################### draw circle
# image, center, radius, color
cv2.circle(pic, (250,250), 150, (255, 0, 0))
cv2.imshow("Image", pic)
cv2.waitKey(0)

# draw a blue 20px (filled in) circle on the image centered at
# x=300,y=150
output = image.copy()
cv2.circle(output, (300, 150), 20, (255, 0, 0), -1)
cv2.imshow("Image.jpg", output)
cv2.waitKey(0)

############################ draw polygon
# define points to plot polygon
pts = np.array([[10, 50], [400, 50], [90, 200], [50, 500]], np.int32)

# reshape as per polylines
pts = pts.reshape((-1, 1, 2))

# image, pnts, isclosed, color, thickness
cv2.polylines(pic, [pts], True, (255, 0, 0), 3)
cv2.imshow("Image", pic)
cv2.waitKey(0)

######################## draw ellipse
image = pic.copy()
# image, center, axes, angle, startAngle, endAngle, color, thickness
cv2.ellipse(image, (150, 150), (150, 150), 30, 0, 360, (0, 255, 255), -1)
cv2.imshow("Image", image)
cv2.waitKey(0)


#################### write text
# image, text, position, font, size, color, thickness
font = cv2.FONT_HERSHEY_DUPLEX
cv2.putText(pic, "Hello", (150, 100), font, 1, (255, 255, 255), 2, cv2.LINE_8)
cv2.imshow("Image", pic)
cv2.waitKey(0)

# draw green text on the image
output = image.copy()
cv2.putText(output, "OpenCV + Abhinav!!!", (10, 25), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
cv2.imshow("Image.jpg", output)
cv2.waitKey(0)

######################################## AFFINE TRANSFORMATIONS ######################
######################### resize
# interpolation: tells how to fill the points when resizing
# cv2.INTER_AREA- Good for shrinking or downsampling
# cv2.INTER_NEAREST- Fastest
# cv2.INTER_LINEAR- Good for zooming or upsampling (default)
# cv2.INTER_CUBIC- Better
# cv2.INTERLANCZOS4- Best
# resize the image to 400x400px, ignoring aspect ratio
# image, (output image size), xscale, yscale, interpolation
resized = cv2.resize(pic, (400, 400))
cv2.imshow("resized", resized)
cv2.waitKey(0)

# fixed resizing and distort aspect ratio so let's resize the width
# to be 400px but compute the new height based on the aspect ratio
(h, w, d) = pic.shape
r = 400.0 / w
dim = (400, int(h * r))
resized = cv2.resize(pic, dim)
cv2.imshow("resized", resized)
cv2.waitKey(0)

# manually computing the aspect ratio can be a pain so let's use the
# imutils library instead
resized = imutils.resize(pic, width=400)
cv2.imshow("imutils resized", resized)
cv2.waitKey(0)

# using pyramids
# half the size of image
smaller = cv2.pyrDown(pic)
cv2.imshow("smaller", smaller)
cv2.waitKey(0)

# double the size of image
larger = cv2.pyrUp(smaller)
cv2.imshow("larger", larger)
cv2.waitKey(0)

##################### shift image
pic = cv2.imread('Abhinav.jpg')
cols = pic.shape[1]
rows = pic.shape[0]
# right by 150, down by 70, M is translation matrix
M = np.float32([[1,0,150], [0,1,70]])
shifted = cv2.warpAffine(pic, M, (cols, rows))
cv2.imshow("shifted", shifted)
cv2.waitKey(0)

############################ rotate image
# let's rotate an image 45 degrees clockwise using OpenCV by first
# computing the image center, then constructing the rotation matrix,
# and then finally applying the affine warp
pic = cv2.imread('Abhinav.jpg')
pic = cv2.resize(pic, (600, 400))
cols = pic.shape[1]
rows = pic.shape[0]
center = (cols//2, rows//2)
angle = -45
scale = 1
M = cv2.getRotationMatrix2D(center, angle, scale)
rotated = cv2.warpAffine(pic, M, (cols, rows))
cv2.imshow("rotated", rotated)
cv2.waitKey(0)

# transpose
rotated = cv2.transpose(pic)
cv2.imshow("Imutils rotated", rotated)
cv2.waitKey(0)

# rotation can also be easily accomplished via imutils with less code
rotated = imutils.rotate(pic, -45)
cv2.imshow("Imutils rotated", rotated)
cv2.waitKey(0)

# OpenCV doesn't "care" if our rotated image is clipped after rotation
# so we can instead use another imutils convenience function to help
# us out
rotated = imutils.rotate_bound(pic, 45)
cv2.imshow("bounded", rotated)
cv2.waitKey(0)


######################## flip image
pic = cv2.imread('Abhinav.jpg')
image = cv2.resize(pic, (600, 400))
output = image.copy()
hf = cv2.flip(image, 0)
cv2.imshow("Horizontally flipped", hf)
cv2.waitKey(0)

vf = cv2.flip(image, 1)
cv2.imshow("Vertically flipped", vf)
cv2.waitKey(0)

bf = cv2.flip(image, -1) 
cv2.imshow("Horizontally and Vertically flipped", bf)
cv2.waitKey(0)

################### perspective affine transform
pic = cv2.imread('test4.jpg')
cv2.imshow('original', pic[73:236, 95:533])

# coordinates of the 4 corners of the original image
# top-left, top-right, bottom-left, bottom-right
points_A = np.float32([[105, 73], [533, 156], [95, 203], [523, 236]])

# coordinates of the 4 corners of the desired output
points_B = np.float32([[0, 0], [450, 0], [0, 120], [450, 120]])

# use the two sets of four points to compute the
# Perspective Transformation Matrix, M
M = cv2.getPerspectiveTransform(points_A, points_B)
warped = cv2.warpPerspective(pic, M, (450, 120))

cv2.imshow('warpPerspective', warped)
cv2.waitKey(0)

#################### Affine Transform
pic = cv2.imread('test4.jpg')
cv2.imshow('original', pic[73:236, 95:533])

# coordinates of the 3 corners of the original image
# top-left, top-right, bottom-left
points_A = np.float32([[105, 73], [533, 156], [95, 203]])

# coordinates of the 3 corners of the desired output
points_B = np.float32([[0, 0], [450, 0], [0, 120]])

# use the two sets of three points to compute the
# Affine Transformation Matrix, M
M = cv2.getAffineTransform(points_A, points_B)
warped = cv2.warpAffine(pic, M, (450, 120))

cv2.imshow('warpAffine', warped)
cv2.waitKey(0)
############################################################################################

#################################### grayscale
gray = cv2.cvtColor(pic, cv2.COLOR_BGR2GRAY)
cv2.imshow("Image.jpg", gray)
cv2.waitKey(0)

###################### image thresholding
# read grayscale image
pic = cv2.imread('Abhinav.jpg', 0)
pic = cv2.resize(pic, (500, 400))

# cv2.THRESH_TOZERO: convert values below threshold to 0 rest will be unchanged
# cv2.THRESH_TRUNC: truncate values above threshold to threshold values and rest will be unchanged
# cv2.THRESH_TOZERO_INV: convert values above threshold to 0 rest will be unchanged
# cv2.THRESH_BINARY: convert values above threshold value to 255 and below to 0
# cv2.THRESH_BINARY_INV: convert values above threshold value to 0 and below to 255

# threshold the image by setting all pixel values less than 100
# to 0 (black; foreground) and all pixel values >= 100 to 255
# (white; background), thereby segmenting the image
threshold_value = 100
(T_value, binary_threshold) = cv2.threshold(pic, threshold_value, 255, cv2.THRESH_BINARY)            
cv2.imshow("image thresholding", binary_threshold)                           
cv2.waitKey(0)

# ADAPTIVE THRESHOLDING
# cv2.THRESH_OTSU: when there are two histogram peaks in the image
thresh = cv2.adaptiveThreshold(pic, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 3, 5)            
cv2.imshow("image thresholding", thresh)                           
cv2.waitKey(0)

########################## image blurring/ removing noise
pic = cv2.imread('Abhinav.jpg')

# convolution
kernel = np.ones((3,3), np.float32)/9
blur = cv2.filter2D(pic, -1, kernel)
cv2.imshow("image blurring", blur)                           
cv2.waitKey(0)

# averaging
blur = cv2.blur(pic, (3,3))
cv2.imshow("image blurring", blur)                           
cv2.waitKey(0)

# apply a Gaussian blur with a 7x7 kernel to the image to smooth it,
# useful when reducing high frequency noise
matrix = (7,7)
# image, filter size, standard deviation
blur = cv2.GaussianBlur(pic, matrix, 0)
cv2.imshow("image blurring", blur)                           
cv2.waitKey(0)

# median blur
kernal = 3
median = cv2.medianBlur(pic, kernal)
cv2.imshow("image blurring", median)
cv2.waitKey(0)

# keeping edges sharp
bilateral = cv2.bilateralFilter(pic, 9, 75, 75)
cv2.imshow("image blurring", bilateral)
cv2.waitKey(0)

# Fast Mean Denoising
# there are other variations of this method
dst = cv2.fastNlMeansDenoisingColored(pic, None, 6, 6, 7, 21)
cv2.imshow("Fast Mean Denoising", dst)
cv2.waitKey(0)

############################## image sharpening
pic = cv2.imread('Abhinav.jpg')

# convolution
kernel = np.array([[-1, -1, -1],
                   [-1, 9, -1],
                   [-1, -1, -1]])
sharpened = cv2.filter2D(pic, -1, kernel)
cv2.imshow("image sharpening", sharpened)                           
cv2.waitKey(0)

###################### canny edge detector
# grayscale image
pic = cv2.imread('Abhinav.jpg', 0)

# applying edge detection we can find the outlines of objects in
# images
threshold1 = 75
threshold2 = 150
canny = cv2.Canny(pic, threshold1, threshold2)
cv2.imshow("canny edge detector", canny)                           
cv2.waitKey(0)

################################ histogram equalization
image_data = cv2.imread('Abhinav.jpg')
# Y = luma component
# UV = chroma component
img_to_yuv = cv2.cvtColor(image_data, cv2.COLOR_BGR2YUV)
img_to_yuv[:,:,0] = cv2.equalizeHist(img_to_yuv[:,:,0])
image_data = cv2.cvtColor(img_to_yuv, cv2.COLOR_YUV2BGR)
cv2.imshow("histogram equalization", image_data)                           
cv2.waitKey(0)

#################################### brightness
# try with different gamma values
gamma=2
invGamma = 1.0 / gamma
table = np.array([((i / 255.0) ** invGamma) * 255
       for i in np.arange(0, 256)]).astype("uint8")
 
# apply gamma correction using the lookup table
image_data = cv2.LUT(image_data, table)
cv2.imshow("brightness", image_data)                           
cv2.waitKey(0)

################################### border
# apply border in the image
# top, bottom, left, right - border width in number of pixels in corresponding directions
# borderType - It can be following types:
# cv2.BORDER_CONSTANT - Adds a constant colored border. The value should be given as next argument.
# cv2.BORDER_REFLECT - Border will be mirror reflection of the border elements, like this : fedcba|abcdefgh|hgfedcb
# cv2.BORDER_REFLECT_101 or cv2.BORDER_DEFAULT - Same as above, but with a slight change, like this : gfedcb|abcdefgh|gfedcba
# cv2.BORDER_REPLICATE - Last element is replicated throughout, like this: aaaaaa|abcdefgh|hhhhhhh
# cv2.BORDER_WRAP - Can’t explain, it will look like this : cdefgh|abcdefgh|abcdefg
# value - Color of border if border type is cv2.BORDER_CONSTANT
border = cv2.copyMakeBorder(gray, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255)
border = cv2.resize(border, (400, 400))
cv2.imshow("border", border)                           
cv2.waitKey(0)

################################ Contour
# read grayscale image
pic = cv2.imread('basic-shapes.jpg')
gray = cv2.cvtColor(pic, cv2.COLOR_BGR2GRAY)
binary_threshold = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)[1]         

# find contours (i.e., outlines) of the foreground objects in the
# thresholded image
# RETR_EXTERNAL- to fetch only external/outer contour
# RETR_LIST - to fetch all the contours
# RETR_CCOMP - to fetch all the contours in 2-level hierarchy
# RETR_TREE - to fetch all the contours in full hierarchy
# CHAIN_APPROX_SIMPLE - To find just the boundary points like two points for a straight line
# CHAIN_APPROX_NONE - To get all the points

cnts = cv2.findContours(binary_threshold.copy(), cv2.RETR_LIST,
                        cv2.CHAIN_APPROX_NONE)
cnts = imutils.grab_contours(cnts)

# 1: without sorting
output = pic.copy()
# loop over the contours
for c in cnts:
    # draw each contour on the output image with a 3px thick purple
    # outline, then display the output contours one at a time
    cv2.drawContours(output, [c], -1, (240, 0, 159), 3)
    cv2.imshow("Contours", output)
    cv2.waitKey(0)
    
    
# 2: After sorting
sorted_contours = sorted(cnts, key = cv2.contourArea, reverse=True)
output = pic.copy()
# loop over the contours
for c in sorted_contours:
    # draw each contour on the output image with a 3px thick purple
    # outline, then display the output contours one at a time
    cv2.drawContours(output, [c], -1, (240, 0, 159), 3)
    cv2.imshow("Contours Sorted", output)
    cv2.waitKey(0)
    

# 3: plot contour center
output = pic.copy()
# loop over the contours
for c in sorted_contours:
    M = cv2.moments(c)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    # draw each contour on the output image with a 3px thick purple
    # outline, then display the output contours one at a time
    cv2.circle(output, (cx, cy), 5, (0, 0, 0), 2)
    cv2.imshow("Contours Sorted", output)
    cv2.waitKey(0)

    
# 4: sort contour center wise/ left to right/ top to bottom
def x_cord_centroid(c):
    M = cv2.moments(c)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return cx

sorted_x_contours = sorted(cnts, key=x_cord_centroid, reverse=False)
# loop over the contours
for c in sorted_x_contours:
    # draw each contour on the output image with a 3px thick purple
    # outline, then display the output contours one at a time
    cv2.drawContours(output, [c], -1, (240, 0, 159), 3)
    cv2.imshow("Contours Sorted on x", output)
    cv2.waitKey(0)

    
# 5: Plot Bounding Rectangle
output = pic.copy()
# loop over the contours
for c in cnts:
    (x, y, w, h) = cv2.boundingRect(c)
    cv2.rectangle(output, (x, y), (x+w, y+h), (0, 0, 255), 2)
    cv2.imshow("Contours Rectangle", output)
    cv2.waitKey(0)


# 6: Approximate polygon shape
output = pic.copy()
# loop over the contours
for c in cnts:
    # contour, approximationAccuracy, closed
    # approximation Accuracy - should be less than 5% of contour perimeter
    accuracy = 0.03 * cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, accuracy, True)
    cv2.drawContours(output, [approx], -1, (240, 0, 159), 3)
    cv2.imshow("Approx Poly DP", output)
    cv2.waitKey(0)
    
    
# 7: Convex Hull
# this basically finds all the outer edges and joins them
output = pic.copy()
# loop over the contours
for c in cnts:
    hull = cv2.convexHull(c)
    cv2.drawContours(output, [hull], -1, (240, 0, 159), 3)
    cv2.imshow("Convex Hull", output)
    cv2.waitKey(0)

    
# 8: Matching Shape
output = pic.copy()
gray = cv2.cvtColor(pic, cv2.COLOR_BGR2GRAY)
threshold = cv2.threshold(gray, 200, 255, 0)[1]         
cnts = cv2.findContours(threshold.copy(), cv2.RETR_CCOMP,
                        cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(cnts)

# collect largest contour as template to be matched
template_contour = cnts[0]
# loop over the contours
for c in cnts:
    # contour template, contour, method, method parameter
    # output value- lower the value better the match
    # template- that we are trying to find in the image
    # method- 1,2,3
    match = cv2.matchShapes(template_contour, c, 1, 0.0)
    print(match)
    if match<0.15:
        closest_contour = c
    cv2.drawContours(output, [c], -1, (240, 0, 159), 3)
    cv2.imshow("output", output)
    cv2.waitKey(0)
    
#################################### Eroding
# we apply erosions to reduce the size of foreground objects
mask = binary_threshold.copy()
mask = cv2.erode(mask, None, iterations=5)
cv2.imshow("Eroded", mask)
cv2.waitKey(0)

################################### Dilating
# similarly, dilations can increase the size of the ground objects
mask = binary_threshold.copy()
mask = cv2.dilate(mask, None, iterations=5)
cv2.imshow("Dilated", mask)
cv2.waitKey(0)

##################################### masking
# a typical operation we may want to apply is to take our mask and
# apply a bitwise AND to our input image, keeping only the masked
# regions
mask = binary_threshold.copy()
output = cv2.bitwise_and(pic, pic, mask=mask)
cv2.imshow("Output", output)
cv2.waitKey(0)
# and, or, xor, not operations can also be done

####################################### blob detection
# Read image
im = cv2.imread("bottle-caps.jpg", cv2.IMREAD_GRAYSCALE)

# Set up the detector with default parameters.
detector = cv2.SimpleBlobDetector()

# Detect blobs.
keypoints = detector.detect(im)

# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), 
                                      cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# Show keypoints
cv2.imshow("Keypoints", im_with_keypoints)
cv2.waitKey(0)

# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

# Change thresholds
params.minThreshold = 10;
params.maxThreshold = 200;

# Filter by Area.
params.filterByArea = True
params.minArea = 1500
params.maxArea = 1500

# Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.1 # (1 being the perfect, 0 being the opposite)

# Filter by Convexity
params.filterByConvexity = True
params.minConvexity = 0.87

# Filter by Inertia- Measure of ellipticalness (low being more elliptical, hight being more circular)
params.filterByInertia = True
params.minInertiaRatio = 0.01

# Create a detector with the parameters
detector = cv2.SimpleBlobDetector_create(params)


