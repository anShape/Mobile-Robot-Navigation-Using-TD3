#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os




if __name__ == '__main__':
    rospy.init_node("test_node", anonymous=True)
    
    data = rospy.wait_for_message('/camera/depth/image_raw', Image, timeout=5)
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    cv_image = cv_image * 128
    print(cv_image.shape)

    # dir = r"/home/ihsan/Data"

    # print(cv_image)

    # os.chdir(dir)

    # filename = 'savedImage.png'
    # cv2.imwrite(filename, cv_image) 
