# Original by: Ihsan Nurkhotib
# If you edit this code, please place your name below

#!/usr/bin/env python3

import rospy 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import GetModelState
import random

def main():
    rospy.init_node('get_pose')

    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    respp1 = get_state('obstacle_1', 'world')
    respp2 = get_state('obstacle_2', 'world')
    respp3 = get_state('obstacle_3', 'world')

    print(respp1)
    print(respp2)
    print(respp3)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass