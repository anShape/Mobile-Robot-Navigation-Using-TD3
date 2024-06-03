import requests
import math
import json
import time
from PIL import Image
from io import BytesIO
import cv2

# bumper = requests.get('http://192.168.0.101/data/bumper')
# print(bumper.json())
# {'value' : False}
# data = bumper.json()
# print(data['value'])

# for i in range(1000):
#     distance = requests.get('http://192.168.0.101/data/distancesensorarray')
#     print(distance.json())
    # [0, 0, 0, 0, 0, 0, 0, 0, 0]
    # print(type(distance.json()))
    # data = distance.json()
    # print(data[0])

# odom = requests.get('http://192.168.0.101/data/odometry')
# print(odom.json())
# [0, 0, 0, 0, 0, 0, 0, 0]

# data = bumper.json()
# print(data['value'])

# yaw = 1
# vl = 0.2
# vw = 0.2

# vx = vl*math.cos(yaw)
# vy = vl*math.sin(yaw)
# om = vw

# data = [0.2, 0, 0]

vl = 0.2
om = 0

yaw = 0
vx = vl*math.cos(yaw)
vy = vl*math.sin(yaw)

data = [vx, vy, om]

for i in range(10):
    r = requests.post('http://192.168.0.101/data/omnidrive', json=data)
    

# data = bumper.json()
# print(data['value'])


