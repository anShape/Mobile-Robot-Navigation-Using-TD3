#!/usr/bin/env python3

import rospy 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import random
import time

def main():
    rospy.init_node('set_pose')

    state_msg_obs1 = ModelState()
    state_msg_obs1.model_name = 'obstacle_1'
    state_msg_obs1.pose.position.z = 0.18
    state_msg_obs1.pose.orientation.x = 0
    state_msg_obs1.pose.orientation.y = 0
    state_msg_obs1.pose.orientation.z = 0
    state_msg_obs1.pose.orientation.w = 0

    state_msg_obs2 = ModelState()
    state_msg_obs2.model_name = 'obstacle_2'
    state_msg_obs2.pose.position.z = 0.18
    state_msg_obs2.pose.orientation.x = 0
    state_msg_obs2.pose.orientation.y = 0
    state_msg_obs2.pose.orientation.z = 0
    state_msg_obs2.pose.orientation.w = 0

    state_msg_obs3 = ModelState()
    state_msg_obs3.model_name = 'obstacle_3'
    state_msg_obs3.pose.position.z = 0.18
    state_msg_obs3.pose.orientation.x = 0
    state_msg_obs3.pose.orientation.y = 0
    state_msg_obs3.pose.orientation.z = 0
    state_msg_obs3.pose.orientation.w = 0

    for i in range(10):
        state_msg_obs1.pose.position.x, state_msg_obs1.pose.position.y, state_msg_obs2.pose.position.x, state_msg_obs2.pose.position.y, state_msg_obs3.pose.position.x, state_msg_obs3.pose.position.y = get_rand_xy(i)
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp1 = set_state( state_msg_obs1 )
            resp2 = set_state( state_msg_obs2 )
            resp3 = set_state( state_msg_obs3 )
            print("Obstacle pose set")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        time.sleep(1)

def get_rand_xy(ran):

    obstacle_pose = [[[0,0],[-0.3,0],[0,0.3]],[[-0.5,-0.5],[0.2,0.5],[0.1,-0.3]],[[-0.5,0],[-1,0],[0,0]],
                              [[1,0],[0.5,0],[0,0]],[[0,0.3],[0,-0.5],[0,0]],[[0,0],[0,0.5],[0,1]],
                              [[0,-0.5],[0.5,0],[-0.5,0]],[[-0.5,-5],[-1,-1],[1,1]],[[-0.1,-0.1],[-0.1,0.1],[0.1,0.1]],
                              [[0.3,-0.7],[-0.4,0],[-0.2,-0.7]]]
    
    # ran = random.randint(0, 9)
    

    x1 = obstacle_pose[ran][0][0]
    y1 = obstacle_pose[ran][0][1]
    x2 = obstacle_pose[ran][1][0]
    y2 = obstacle_pose[ran][1][1]
    x3 = obstacle_pose[ran][2][0]
    y3 = obstacle_pose[ran][2][1]


    return x1, y1, x2, y2, x3, y3

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass