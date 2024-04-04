import cv2
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

bag = rosbag.Bag('/home/ihsan/bagfiles/2024-03-30-13-23-37.bag', "r")
count = 0
for topic, msg, timestamp in bag.read_messages(topics=['/camera/depth/image_raw', '/camera/rgb/image_raw']):
    if topic == '/camera/depth/image_raw':
        depth_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv2.imwrite('depth_{}.raw'.format(count), depth_img)

    if topic == '/camera/rgb/image_raw':
        color_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite('depth_{}.jpg'.format(count), color_img)

    count += 1