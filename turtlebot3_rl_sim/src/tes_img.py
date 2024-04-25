# importing cv2  
import cv2 
  
# importing os module   
import os 
  
# Image path 
image_path = r'/home/ihsan/Data/savedImage.png'
  
# Image directory 
directory = r'/home/ihsan/Data'
  
# Using cv2.imread() method 
# to read the image 
img = cv2.imread(image_path, cv2.IMREAD_UNCHANGED) 

print(img.shape)