## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#####################################################
## librealsense tutorial #1 - Accessing depth data ##
#####################################################

# First import the library
import pyrealsense2 as rs
import numpy as np
from matplotlib import pyplot as plt

import numpy as np
from PIL import Image
import time


# Sample 1D NumPy array
array = np.random.rand(100) * 255  # Example array with values in the range [0, 255]




try:
    # Create a context object. This object owns the handles to all connected realsense devices
    pipeline = rs.pipeline()

    # Configure streams
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Start streaming
    pipeline.start(config)

    haha = True

    while haha == True:
        # This call waits until a new coherent set of frames is available on a device
        # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        # depth_image = depth_image * depth_frame.get_units()


        # depth_image_meters = np.asanyarray(depth.get_data())
        # depth_image_meters = depth_image_meters * depth.get_units()
        # colorizer = rs.colorizer()
        # colorized_depth = np.asanyarray(colorizer.colorize(depth).get_data())

        # plt.rcParams["axes.grid"] = False
        # plt.rcParams['figure.figsize'] = [8, 4]
        # plt.imshow(colorized_depth)

        colorizer = rs.colorizer()
        # colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())

        decimation = rs.decimation_filter()
        depth_frame = decimation.process(depth_frame)

        spatial = rs.spatial_filter()
        depth_frame = spatial.process(depth_frame)
        # colorized_depth = np.asanyarray(colorizer.colorize(filtered_depth).get_data())
        # plt.imshow(colorized_depth)

        hole_filling = rs.hole_filling_filter(2)
        filled_depth = hole_filling.process(depth_frame)
        colorized_depth = np.asanyarray(colorizer.colorize(filled_depth).get_data())
        # plt.imshow(colorized_depth)

        plt.rcParams["axes.grid"] = False
        plt.rcParams['figure.figsize'] = [8, 4]
        plt.imshow(colorized_depth)
        plt.show()
        # decimation = rs.decimation_filter()
        # spatial = rs.spatial_filter()
        # temporal = rs.temporal_filter()
        # hole_filling = rs.hole_filling_filter()
        # # depth_to_disparity = rs.disparity_transform(True)
        # # disparity_to_depth = rs.disparity_transform(False)

        
        # frame = depth_image_meters
        # frame = decimation.process(frame)
        # # frame = depth_to_disparity.process(frame)
        # frame = spatial.process(frame)
        # frame = temporal.process(frame)
        # # frame = disparity_to_depth.process(frame)
        # frame = hole_filling.process(frame)
        print("euyy")

        # colorized_depth = np.asanyarray(colorizer.colorize(frame).get_data())
        # print("euy")
        # plt.imshow(colorized_depth)
        # print("walah")

        # np.savetxt('image_data.txt', depth_image_meters, fmt='%.6f')
        # print("saved")
        time.sleep(5)
        # print(depth_image_meters)

        # Normalize if necessary (only needed if your array values are not already in [0, 255])
        # depth_image_meters = np.clip(depth_image_meters, 0, 255)

        # # Reshape the array to a 2D format if needed (e.g., 10x10 image)
        # array_2d = depth_image_meters.reshape((480, 640))

        # # Convert to an unsigned 8-bit integer type (common for images)
        # array_2d = array_2d.astype(np.uint8)

        # # Create an image from the array
        # image = Image.fromarray(array_2d)

        # # Save the image
        # image.save('depth_image_raw.png')

        # # Display the image (optional)
        # image.show()
        # plt.imshow(depth_image, interpolation='nearest')
        # plt.show()
        # print("walah")

        # haha = False
        # print(depth_image_meters)

        # Print a simple text-based representation of the image, by breaking it into 10x20 pixel regions and approximating the coverage of pixels within one meter
        # coverage = [0]*64
        # for y in range(480):
        #     for x in range(640):
        #         dist = depth.get_distance(x, y)
        #         if 0 < dist and dist < 1:
        #             coverage[x//10] += 1
            
        #     if y%20 is 19:
        #         line = ""
        #         for c in coverage:
        #             line += " .:nhBXWW"[c//25]
        #         coverage = [0]*64
        #         print(line)
    exit(0)
#except rs.error as e:
#    # Method calls agaisnt librealsense objects may throw exceptions of type pylibrs.error
#    print("pylibrs.error was thrown when calling %s(%s):\n", % (e.get_failed_function(), e.get_failed_args()))
#    print("    %s\n", e.what())
#    exit(1)
except Exception as e:
    print(e)
    pass