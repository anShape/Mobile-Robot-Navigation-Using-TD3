#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image

if __name__ == '__main__':
    rospy.init_node("test_node", anonymous=True)
    
    data = rospy.wait_for_message('/camera/depth/image_raw', Image, timeout=5)
    # data = [1,2,3]
    print("tes", data)