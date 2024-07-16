## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

# Edited by: Ihsan Nurkhotib
# If you edit this code, please place your name below

import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
from matplotlib import pyplot as plt
import numpy as np
import skimage.exposure

# data_cam = rospy.wait_for_message('camera/depth/image_raw', Image, timeout=5)
# bridge = CvBridge()
# data_cam = bridge.imgmsg_to_cv2(data_cam, desired_encoding='passthrough')
# print(data_cam.shape)
rospy.init_node('td3_training', anonymous=True)

while(1):
    data_cam = rospy.wait_for_message('/camera/depth/image_raw', Image, timeout=5)
    bridge = CvBridge()
    data_cam = bridge.imgmsg_to_cv2(data_cam, desired_encoding='passthrough')


    # load image as grayscale
    # img = cv2.imread("dist_img.png", cv2.IMREAD_ANYDEPTH)

    # stretch to full dynamic range
    stretch = skimage.exposure.rescale_intensity(data_cam, in_range='image', out_range=(0,255)).astype(np.uint8)

    # convert to 3 channels
    stretch = cv2.merge([stretch,stretch,stretch])

    # define colors
    color10 = (0, 0, 255)     #red
    color9 = (0, 30, 255)     #red
    color8 = (0, 70, 255)     #red
    color7 = (0, 100, 255)     #red
    color6 = (0, 165, 255)   #orange
    color5 = (0, 200, 255)   #orange
    color4 = (0, 255, 255)   #yellow
    color3 = (255, 255, 0)   #cyan
    color2 = (255, 0, 0)     #blue
    colorArr = np.array([[color2, color3, color4, color5, color6, color7, color8, color9, color10]], dtype=np.uint8)

    # resize lut to 256 (or more) values
    lut = cv2.resize(colorArr, (256,1), interpolation = cv2.INTER_LINEAR)

    # apply lut
    result = cv2.LUT(stretch, lut)

    # create gradient image
    grad = np.linspace(0, 255, 512, dtype=np.uint8)
    grad = np.tile(grad, (20,1))
    grad = cv2.merge([grad,grad,grad])

    # apply lut to gradient for viewing
    grad_colored = cv2.LUT(grad, lut)

    # save result
    cv2.imwrite('dist_img_colorized.png', result)
    cv2.imwrite('dist_img_lut.png', grad_colored)

    # display result
    cv2.imshow('RESULT', result)
    cv2.imshow('LUT', grad_colored)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Configure depth and color streams
# pipeline = rs.pipeline()
# config = rs.config()

# # Get device product line for setting a supporting resolution
# pipeline_wrapper = rs.pipeline_wrapper(pipeline)
# pipeline_profile = config.resolve(pipeline_wrapper)
# device = pipeline_profile.get_device()
# device_product_line = str(device.get_info(rs.camera_info.product_line))

# found_rgb = False
# for s in device.sensors:
#     if s.get_info(rs.camera_info.name) == 'RGB Camera':
#         found_rgb = True
#         break
# if not found_rgb:
#     print("The demo requires Depth camera with Color sensor")
#     exit(0)

# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# # Start streaming
# pipeline.start(config)

# try:
#     while True:

#         # Wait for a coherent pair of frames: depth and color
#         frames = pipeline.wait_for_frames()
#         depth_frame = frames.get_depth_frame()
#         # color_frame = frames.get_color_frame()
#         if not depth_frame:
#             continue

#         # Convert images to numpy arrays
#         depth_image = np.asanyarray(depth_frame.get_data())
#         # color_image = np.asanyarray(color_frame.get_data())

#         # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
#         # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

#         # depth_colormap_dim = depth_colormap.shape
#         # # color_colormap_dim = color_image.shape

#         # # If depth and color resolutions are different, resize color image to match depth image for display
#         # if depth_colormap_dim != color_colormap_dim:
#         #     resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
#         #     images = np.hstack((resized_color_image, depth_colormap))
#         # else:
#         #     images = np.hstack((color_image, depth_colormap))

#         # Show images
#         # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
#         # cv2.imshow('RealSense', images)
#         # cv2.waitKey(1)
#         print(depth_image.shape)

# finally:

#     # Stop streaming
#     pipeline.stop()