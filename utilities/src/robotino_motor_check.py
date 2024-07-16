# Original by: Ihsan Nurkhotib
# If you edit this code, please place your name below

import requests
import math
import json
import time
import rospy
import signal
import sys

deg = -1.5

# def main():

#     while True:
#         data_odom = requests.get('http://192.168.0.101/data/odometry')
#         data_odom = data_odom.json()
#         print(data_odom)

#         data_odom[0] = (data_odom[0] - 1) * -1 # disesuaikan dengan koordinat gazebo
#         data_odom[1] = (data_odom[1] + 1) * -1 
#         if data_odom[2] > 0:
#             data_odom[2] = data_odom[2] - math.pi
#         else:
#             data_odom[2] = data_odom[2] + math.pi

#         if data_odom[2] < 1.6 and data_odom[2] > 1.5:
#             vl = 1
#             vw = 0
#         elif data_odom[2] > 1.6 or data_odom[2] < -1.5:
#             vl = 0
#             vw = 0.5
#         else:
#             vl = 0
#             vw = -0.5

#         print(data_odom)

#         if data_odom[2] > 0:
#             data_odom[2] = data_odom[2] - math.pi
#         else:
#             data_odom[2] = data_odom[2] + math.pi

#         vx = vl*math.cos(data_odom[2])
#         vy = vl*math.sin(data_odom[2])

#         data = [vx, vy, -vw]
        
#         r = requests.post('http://192.168.0.101/data/omnidrive', json=data)
        
def main():

    while True:
        data_odom = requests.get('http://192.168.0.101/data/odometry')
        data_odom = data_odom.json()
        print("ODOM: ", data_odom)

        if data_odom[2] < deg + 0.1 and data_odom[2] > deg - 0.1:
            vl = 0.3
            vw = 0
        elif data_odom[2] > deg + 0.1 or data_odom[2] < deg:
            vl = 0
            vw = -0.5
        else:
            vl = 0
            vw = 0.5


        # vl = 0.3
        # vw = 0

        # data = [vx, vy, vw]
        data = [vl, 0, vw]
        print("DATA: ", data)

        # data = [0, 0, -1]

        # for i in range(10):
        #     r = requests.post('http://192.168.0.101/data/omnidrive', json=data)

        
        r = requests.post('http://192.168.0.101/data/omnidrive', json=data)

def signal_handler(sig, frame):
    print('Menghentikan program...')
    rospy.signal_shutdown('SIGINT diterima')
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('td3_test', anonymous=True)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        print('Keyboard Interrupted')
        requests.post('http://192.168.0.101/data/omnidrive', json=[0,0,0])    