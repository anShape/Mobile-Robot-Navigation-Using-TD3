#!/usr/bin/env python3
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# Edited by: Ihsan Nurkhotib
# If you edit this code, please place your name below

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        # cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        # cv2_img = bridge.imgmsg_to_cv2(msg, "32FC1")
        cv2_img = bridge.imgmsg_to_cv2(msg, "passthrough")
        cv2_img = cv2.normalize(cv2_img, cv2_img, 0, 1, cv2.NORM_MINMAX)
        print("ini printatn", cv2_img)
    except CvBridgeError as e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite('camera_image_640x480.png', cv2_img)
    

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/camera/depth/image_raw"
    # image_topic = "/camera/rgb/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()