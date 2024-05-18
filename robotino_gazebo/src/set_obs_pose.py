#!/usr/bin/env python3

import rospy 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import random

def main():
    rospy.init_node('set_pose')

    obs_pose = []


    state_msg_obs1 = ModelState()
    state_msg_obs1.model_name = 'obstacle_1'
    state_msg_obs1.pose.position.x, state_msg_obs1.pose.position.y = 0,0
    state_msg_obs1.pose.position.z = 0
    state_msg_obs1.pose.orientation.x = 0
    state_msg_obs1.pose.orientation.y = 0
    state_msg_obs1.pose.orientation.z = 0
    state_msg_obs1.pose.orientation.w = 0

    state_msg_obs2 = ModelState()
    state_msg_obs2.model_name = 'obstacle_2'
    state_msg_obs2.pose.position.x, state_msg_obs1.pose.position.y = -0.3, 0
    state_msg_obs2.pose.position.z = 0
    state_msg_obs2.pose.orientation.x = 0
    state_msg_obs2.pose.orientation.y = 0
    state_msg_obs2.pose.orientation.z = 0
    state_msg_obs2.pose.orientation.w = 0

    state_msg_obs3 = ModelState()
    state_msg_obs3.model_name = 'obstacle_3'
    state_msg_obs3.pose.position.x, state_msg_obs1.pose.position.y = 0, 0.3
    state_msg_obs3.pose.position.z = 0
    state_msg_obs3.pose.orientation.x = 0
    state_msg_obs3.pose.orientation.y = 0
    state_msg_obs3.pose.orientation.z = 0
    state_msg_obs3.pose.orientation.w = 0

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp1 = set_state( state_msg_obs1 )
        resp2 = set_state( state_msg_obs2 )
        resp3 = set_state( state_msg_obs3 )
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass