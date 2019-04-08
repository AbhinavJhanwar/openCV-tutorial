# -*- coding: utf-8 -*-
"""
Created on Thu Jan 10 15:50:43 2019

@author: abhinav.jhanwar
"""

import cv2
import numpy as np

################ create image with black background
# color, data type
pic = np.zeros((500, 500, 3), dtype='uint8')

################## draw rectangle
# image, initial pos, final pos, color, width
cv2.rectangle(pic, (0,0), (500, 150), (255, 0, 0), 3, lineType=8, shift=0)

# show image
cv2.imshow("Image", pic)
cv2.waitKey(0)

################ draw line
# image, initial pos, final pos, color
cv2.line(pic, (0,0), (500, 150), (255, 0, 0), 5)
cv2.imshow("Image", pic)
cv2.waitKey(0)

###################### draw circle
# image, center, radius, color
cv2.circle(pic, (250,250), 150, (255, 0, 0))
cv2.imshow("Image", pic)
cv2.waitKey(0)

#################### write text
# image, text, position, size, color, thickness
font = cv2.FONT_HERSHEY_DUPLEX
cv2.putText(pic, "Hello", (150, 100), font, 1, (255, 255, 255), 2, cv2.LINE_8)
cv2.imshow("Image", pic)
cv2.waitKey(0)

##################### shift image
pic = cv2.imread('Abhinav.jpg')
cols = pic.shape[1]
rows = pic.shape[0]
# right by 150, down by 70
M = np.float32([[1,0,150], [0,1,70]])
shifted = cv2.warpAffine(pic, M, (cols, rows))
cv2.imshow("shifted", shifted)
cv2.waitKey(0)

################### rotate image
pic = cv2.imread('Abhinav.jpg')
cols = pic.shape[1]
rows = pic.shape[0]
center = (cols/2, rows/2)
angle = 10
scale = 1
M = cv2.getRotationMatrix2D(center, angle, scale)
rotated = cv2.warpAffine(pic, M, (cols, rows))
cv2.imshow("rotated", rotated)
cv2.waitKey(0)

###################### image thresholding
# read grayscale image
pic = cv2.imread('Abhinav.jpg', 0)
threshold_value = 60
(T_value, binary_threshold) = cv2.threshold(pic, threshold_value, 255, cv2.THRESH_BINARY)            
cv2.imshow("image thresholding", binary_threshold)                           
cv2.waitKey(0)

########################## image blurring
pic = cv2.imread('Abhinav.jpg')
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

###################### canny edge detector
pic = cv2.imread('Abhinav.jpg', 0)
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


