import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import skimage.exposure

rospy.init_node('td3_training', anonymous=True)

while(1):
    data_cam = rospy.wait_for_message('/camera/depth/image_raw', Image, timeout=5)
    bridge = CvBridge()
    data_cam = bridge.imgmsg_to_cv2(data_cam, desired_encoding='passthrough')

    # Stretch to full dynamic range
    stretch = skimage.exposure.rescale_intensity(data_cam, in_range='image', out_range=(0,255)).astype(np.uint8)

    # Convert to 3 channels
    stretch = cv2.merge([stretch,stretch,stretch])

    # Define colors with adjusted ranges
    color6 = (0, 0, 255)     # Red
    color5 = (0, 165, 255)   # Orange
    color4 = (0, 255, 255)   # Yellow
    color3 = (255, 255, 0)   # Cyan
    color2 = (255, 0, 0)     # Blue

    # Adjust the size of each color range
    # Blue will now only cover the first 20% of the range
    colorArr = np.zeros((1, 256, 3), dtype=np.uint8)
    colorArr[:, :51] = color2    # 20% for blue (51 values)
    colorArr[:, 51:102] = color3 # 20% for cyan (51 values)
    colorArr[:, 102:153] = color4 # 20% for yellow (51 values)
    colorArr[:, 153:204] = color5 # 20% for orange (51 values)
    colorArr[:, 204:] = color6   # 20% for red (52 values)

    # Apply LUT
    result = cv2.LUT(stretch, colorArr)

    # Create gradient image
    grad = np.linspace(0, 255, 512, dtype=np.uint8)
    grad = np.tile(grad, (20,1))
    grad = cv2.merge([grad,grad,grad])

    # Apply LUT to gradient for viewing
    grad_colored = cv2.LUT(grad, colorArr)

    # Save result
    cv2.imwrite('dist_img_colorized.png', result)
    cv2.imwrite('dist_img_lut.png', grad_colored)

    # Display result
    cv2.imshow('RESULT', result)
    cv2.imshow('LUT', grad_colored)
    cv2.waitKey(0)
    cv2.destroyAllWindows()