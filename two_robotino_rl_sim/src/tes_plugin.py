import utils
import rospy
# from std_msgs.msg import LaserScan, Image
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge

data_laser = rospy.wait_for_message('scan', LaserScan, timeout=5)
data_bumper = utils.get_bumper_data()
data_cam = rospy.wait_for_message('camera/depth/image_raw', Image, timeout=5)
bridge = CvBridge()
data_cam = bridge.imgmsg_to_cv2(data_cam, desired_encoding='passthrough')


print("Laser data: ", data_laser)
print("Bumper data: ", data_bumper)
print("Camera data: ", data_cam)
