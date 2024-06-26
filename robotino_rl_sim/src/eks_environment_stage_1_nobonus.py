#!/usr/bin/env python3
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #
# Added: Risk Perception of the Moving Crowd (by Hafiq Anas) #

import rospy
import numpy as np
import math
import time
from math import pi
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from gazebo_msgs.msg import ContactsState
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge

from collections import deque
from uuid import uuid4
from itertools import chain
import utils

from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState


class Env:
    def __init__(self, action_dim=2, max_step=200):
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.get_odometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.action_dim = action_dim
        # Keys CTRL + c will stop script
        rospy.on_shutdown(self.shutdown)

        # Added
        self.k_obstacle_count = 3
        self.obstacle_present_step_counts = 0
        self.ego_score_collision_prob = 0.0
        self.vel_cmd = 0.0
        self.orientation = 0.0
        self.previous_yaw = 3.14
        self.robot_yaw = 3.14
        self.linear_twist = 0.0
        self.angular_twist = 0.0
        self.previous_heading = 0.0
        self.previous_distance = 0.0
        self.episode_success = False
        self.episode_failure = False
        self.social_safety_violation_count = 0
        self.ego_safety_violation_count = 0
        self.starting_point = Point()
        self.starting_point.x = rospy.get_param("/robotino/starting_pose/x")
        self.starting_point.y = rospy.get_param("/robotino/starting_pose/y")
        self.starting_point.z = rospy.get_param("/robotino/starting_pose/z")

        self.prev_pos = Point()
        self.prev_pos.x = rospy.get_param("/robotino/starting_pose/x")
        self.prev_pos.y = rospy.get_param("/robotino/starting_pose/y")
        self.prev_pos.z = rospy.get_param("/robotino/starting_pose/z")

        self.original_desired_point = Point()
        self.original_desired_point.x = rospy.get_param("/robotino/desired_pose/x")
        self.original_desired_point.y = rospy.get_param("/robotino/desired_pose/y")
        self.original_desired_point.z = rospy.get_param("/robotino/desired_pose/z")

        self.obstacle_pose = [[[0,0],[-0.3,0],[0,0.3]],[[-0.5,-0.5],[0.2,0.5],[0.1,-0.3]],[[-0.5,0],[-1,0],[0,0]],
                              [[1,0],[0.5,0],[0,0]],[[0,0.3],[0,-0.5],[0,0]],[[0,0],[0,0.5],[0,1]],
                              [[0,-0.5],[0.5,0],[-0.5,0]],[[-0.5,-5],[-1,-1],[1,1]],[[-0.1,-0.1],[-0.1,0.1],[0.1,0.1]],
                              [[0.3,-0.7],[-0.4,0],[-0.2,-0.7]]]
        self.obstacle_pose_count = 0

        self.waypoint_desired_point = Point()
        self.waypoint_desired_point.x = self.original_desired_point.x
        self.waypoint_desired_point.y = self.original_desired_point.y
        self.waypoint_desired_point.z = self.original_desired_point.z

        self.linear_forward_speed = rospy.get_param('/robotino/linear_forward_speed')
        self.linear_turn_speed = rospy.get_param('/robotino/linear_turn_speed')
        self.angular_speed = rospy.get_param('/robotino/angular_speed')
        self.scan_ranges = rospy.get_param('/robotino/scan_ranges')
        self.max_scan_range = rospy.get_param('/robotino/max_scan_range')
        self.min_scan_range = rospy.get_param('/robotino/min_scan_range')
        self.max_steps = max_step
        self.done = False

        # Reward shaping based on moving obstacle region and proximity
        self.collision_prob = None
        self.goal_reaching_prob = None
        self.general_collision_prob = None
        self.closest_obstacle_region = None
        self.closest_obstacle_pose = None
        self.closest_obstacle_vel = None
        self.closest_obstacle_pose_vel = None

        # Deque lists to compare items between time steps
        self.agent_pose_deque = deque([])
        self.agent_pose_deque2 = deque([])
        self.obstacle_pose_deque = utils.init_deque_list(self.scan_ranges - 1)
        self.obstacle_pose_deque_list = []
        self.vel_t0 = -1  # Store starting time when vel cmd is executed, to get a time step length
        self.timestep_counter = 0
        self.agent_vel_timestep = 0
        self.filtered_obstacle_pose_deque = None
        self.overall_timesteps = 0.0
        self.previous_known_iou = []

        # Ground truth data
        self.ground_truth_scans = [self.max_scan_range] * (self.scan_ranges - 1)
        self.ground_truth_poses = None
        self.bounding_box_size = None

        # Obstacle Tracking
        self.tracked_obstacles = {}
        self.tracked_obstacles_keys = []
        self.prev_tracked_obstacles = {}
        self.prev_tracked_obstacles_key = []

        # Temporary (delete)
        self.step_reward_count = 0
        self.dtg_reward_count = 0
        self.htg_reward_count = 0
        self.dtg_penalty_count = 0
        self.htg_penalty_count = 0
        self.forward_action_reward_count = 0
        self.left_turn_action_reward_count = 0
        self.right_turn_action_reward_count = 0
        self.weak_left_turn_action_reward_count = 0
        self.weak_right_turn_action_reward_count = 0
        self.strong_left_turn_action_reward_count = 0
        self.strong_right_turn_action_reward_count = 0
        self.rotate_in_place_action_reward_count = 0
        self.stop_action_reward_count = 0
        self.social_nav_reward_count = 0
        self.last_action = "FORWARD"

        self.total_x_travelled = 0
        self.total_y_travelled = 0

        # RVIZ visualization markers to see Collision Probabilities of obstacles w.r.t robot's motion
        self.pub_obs1_pose_text = rospy.Publisher('/obstacle_text_poses/1', Marker, queue_size=1)
        self.pub_obs2_pose_text = rospy.Publisher('/obstacle_text_poses/2', Marker, queue_size=1)
        self.pub_obs3_pose_text = rospy.Publisher('/obstacle_text_poses/3', Marker, queue_size=1)

        self.pub_obs1_pose_shape = rospy.Publisher('/obstacle_text_shape/1', Marker, queue_size=1)
        self.pub_obs2_pose_shape = rospy.Publisher('/obstacle_text_shape/2', Marker, queue_size=1)
        self.pub_obs3_pose_shape = rospy.Publisher('/obstacle_text_shape/3', Marker, queue_size=1)

        # for testing
        self.inc = 1

        # Obstacle rand pose change
        state_msg_obs1 = ModelState()
        state_msg_obs1.model_name = 'obstacle_1'
        state_msg_obs1.pose.position.x, state_msg_obs1.pose.position.y = utils.get_rand_xy()
        state_msg_obs1.pose.position.z = 0
        state_msg_obs1.pose.orientation.x = 0
        state_msg_obs1.pose.orientation.y = 0
        state_msg_obs1.pose.orientation.z = 0
        state_msg_obs1.pose.orientation.w = 0

        self.state_msg_obs1 = state_msg_obs1

        state_msg_obs2 = ModelState()
        state_msg_obs2.model_name = 'obstacle_2'
        state_msg_obs2.pose.position.x, state_msg_obs1.pose.position.y = utils.get_rand_xy()
        state_msg_obs2.pose.position.z = 0
        state_msg_obs2.pose.orientation.x = 0
        state_msg_obs2.pose.orientation.y = 0
        state_msg_obs2.pose.orientation.z = 0
        state_msg_obs2.pose.orientation.w = 0
        
        self.state_msg_obs2 = state_msg_obs2

        state_msg_obs3 = ModelState()
        state_msg_obs3.model_name = 'obstacle_3'
        state_msg_obs3.pose.position.x, state_msg_obs1.pose.position.y = utils.get_rand_xy()
        state_msg_obs3.pose.position.z = 0
        state_msg_obs3.pose.orientation.x = 0
        state_msg_obs3.pose.orientation.y = 0
        state_msg_obs3.pose.orientation.z = 0
        state_msg_obs3.pose.orientation.w = 0

        self.state_msg_obs3 = state_msg_obs3

    def shutdown(self):
        rospy.loginfo("Stopping TurtleBot")
        self.pub_cmd_vel.publish(Twist())
        time.sleep(1)

    def get_robot_obs_xy_diff(self, robot_pose_x, robot_pose_y, obs_pose_x, obs_pose_y):
        """
        Args:
            robot_pose_x: robot's x position
            robot_pose_y: robot's y position
            obs_pose_x: obstacle's x position
            obs_pose_y: obstacle's y position

        Returns: returns distance in x and y axis between robot and obstacle

        """
        robot_obs_x = abs(robot_pose_x - obs_pose_x)
        robot_obs_y = abs(robot_pose_y - obs_pose_y)

        return [robot_obs_x, robot_obs_y]

    def get_distance_from_point(self, pstart, p_end):
        a = np.array((pstart.x, pstart.y, pstart.z))
        b = np.array((p_end.x, p_end.y, p_end.z))

        distance = np.linalg.norm(a - b)

        return distance

    def get_distance_to_goal(self, current_position):
        distance = self.get_distance_from_point(current_position,
                                                self.waypoint_desired_point)
        
        # print("Current position to waypoint distance: " + str(distance))
        # print("Waypoint: [" + str(self.waypoint_desired_point.x) + ", " + str(self.waypoint_desired_point.y) + "]")

        return distance

    def get_actual_distance_to_goal(self, current_position):
        distance = self.get_distance_from_point(current_position,
                                                self.original_desired_point)
        
        # print("Current position to goal distance: " + str(distance))
        # print("Goal: " + str(self.original_desired_point))

        return distance

    def get_angle_from_point(self, current_orientation):
        current_ori_x = current_orientation.x
        current_ori_y = current_orientation.y
        current_ori_z = current_orientation.z
        current_ori_w = current_orientation.w

        orientation_list = [current_ori_x, current_ori_y, current_ori_z, current_ori_w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        return yaw

    def get_heading_to_goal(self, current_position, current_orientation):
        current_pos_x = current_position.x + self.starting_point.x
        current_pos_y = current_position.y + self.starting_point.y

        yaw = self.get_angle_from_point(current_orientation)
        goal_angle = math.atan2(self.waypoint_desired_point.y - current_pos_y,
                                self.waypoint_desired_point.x - current_pos_x)

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        return heading

    def get_odometry(self, odom):
        self.position = odom.pose.pose.position
        self.orientation = odom.pose.pose.orientation
        self.linear_twist = odom.twist.twist.linear
        self.angular_twist = odom.twist.twist.angular

    def get_state(self, data_laser, data_bumper, data_cam, step_counter=0, action=[0, 0]):

        if step_counter == 1:
            # Get updated waypoints according to the Point Of Intersection at circle (Robot FOV)
            goal_waypoints = utils.get_local_goal_waypoints([self.position.x, self.position.y],
                                                            [self.original_desired_point.x,
                                                             self.original_desired_point.y], 0.3)

            self.waypoint_desired_point.x = goal_waypoints[0]
            self.waypoint_desired_point.y = goal_waypoints[1]

        distance_to_goal = round(self.get_distance_to_goal(self.position), 2)
        heading_to_goal = round(self.get_heading_to_goal(self.position, self.orientation), 2)
        actual_distance_to_goal = round(self.get_actual_distance_to_goal(self.position), 2)

        # if step_counter % 5 == 0 or distance_to_goal < self.previous_distance: # original 
        if step_counter % 5 == 0:    
            goal_waypoints = utils.get_local_goal_waypoints([self.position.x, self.position.y],
                                                            [self.original_desired_point.x,
                                                             self.original_desired_point.y], 0.3)

            self.waypoint_desired_point.x = goal_waypoints[0]
            self.waypoint_desired_point.y = goal_waypoints[1]

        agent_vel_x = -1.0 * (self.linear_twist.x * math.cos(self.angular_twist.z))
        agent_vel_y = self.linear_twist.x * math.sin(self.angular_twist.z)
        obstacle_vel_x = 0.0
        obstacle_vel_y = 0.0
        self.closest_obstacle_pose = [self.position.x, self.position.y]
        self.closest_obstacle_vel = [0.0, 0.0]
        self.closest_obstacle_pose_vel = [self.position.x, self.position.y, 0.0, 0.0] * self.k_obstacle_count

        # Get scan ranges from sensor, reverse the scans and remove the final scan because the scan reads in an
        # anti-clockwise manner and the final scan is the same as the first scan, so it is omitted
        _scan_range = utils.get_scan_ranges(data_laser, self.scan_ranges, self.max_scan_range)
        scan_range = _scan_range[:]

        # Get cartesian coordinate of each obstacle poses from the scans.
        yaw = self.get_angle_from_point(self.orientation)
        self.robot_yaw = yaw

        if not self.done:
            if data_bumper[0] == 1:
                print("DONE: MINIMUM RANGE")
                # print("MINIMUM: ", str(min(current_scans)))
                self.done = True

            if self.is_in_true_desired_position(self.position):
                print("DONE: IN DESIRED POSITION")
                self.done = True

            if step_counter >= self.max_steps:
                print("DONE: STEP COUNTER > MAX STEP")
                self.done = True

        agent_position = [round(self.position.x, 3), round(self.position.y, 3)]
        agent_orientation = [round(self.robot_yaw, 3)]
        agent_velocity = [round(agent_vel_x, 3), round(agent_vel_y, 3)]

        goal_heading_distance = [heading_to_goal, distance_to_goal]

        # Image processing
        cnn_result = utils.cnn(data_cam)

        # For inserting goal point to the state
        desired_point = [self.original_desired_point.x*10, self.original_desired_point.y*10]

        state = (scan_range + goal_heading_distance + agent_position + agent_orientation + agent_velocity
                 + cnn_result + data_bumper + desired_point)

        # print("State: ", state)

        # Round items in state to 2 decimal places
        state = list(np.around(np.array(state), 3))

        return state, self.done

    def compute_reward(self, state, step_counter, done):
        current_heading = state[9]
        current_distance = state[10]

        penalty_loop = 0

        # if (step_counter % 75) == 0:
        #     travel_x = self.prev_pos.x - self.position.x
        #     travel_y = self.prev_pos.y - self.position.y
        #     self.prev_pos.x = self.position.x
        #     self.prev_pos.y = self.position.y
        #     if travel_x < 0.5 and travel_y < 0.5:
        #         # print("Robot is stuck in a loop")
        #         rospy.loginfo("Robot is stuck in a loop!")
        #         penalty_loop = -300

        distance_difference = current_distance - self.previous_distance
        heading_difference = current_heading - self.previous_heading

        step_reward = -1  # step penalty
        htg_reward = 0
        dtg_reward = 0
        waypoint_reward = 0

        # Action reward
        # if self.last_action == "FORWARD":
        #     self.forward_action_reward_count += 1 # original 1
        #     action_reward = 10
        # if self.last_action == "TURN_LEFT":
        #     self.left_turn_action_reward_count += 1 # originial 1
        #     action_reward = 1
        # if self.last_action == "TURN_RIGHT":
        #     self.right_turn_action_reward_count += 1 # original 1
        #     action_reward = 1
        # if self.last_action == "STOP":
        #     self.stop_action_reward_count += 1 # original 1
        #     action_reward = 1

        # Distance to goal reward
        if distance_difference > 0:
            self.dtg_penalty_count += 1
            dtg_reward = 0
        if distance_difference < 0:
            self.dtg_reward_count += 1
            dtg_reward = 1

        # Heading to goal reward
        # if heading_difference > 0:
        #     if current_heading > 0 and self.previous_heading < 0:
        #         self.htg_reward_count += 1
        #         htg_reward = 1
        #     if current_heading < 0 and self.previous_heading < 0:
        #         self.htg_reward_count += 1
        #         htg_reward = 1
        #     if current_heading < 0 and self.previous_heading > 0:
        #         self.htg_reward_count += 1
        #         htg_reward = 1
        #     if current_heading > 0 and self.previous_heading > 0:
        #         self.htg_penalty_count += 1
        #         htg_reward = 0
        # if heading_difference < 0:
        #     if current_heading < 0 and self.previous_heading > 0:
        #         self.htg_reward_count += 1
        #         htg_reward = 1
        #     if current_heading > 0 and self.previous_heading > 0:
        #         self.htg_reward_count += 1
        #         htg_reward = 1
        #     if current_heading > 0 and self.previous_heading < 0:
        #         self.htg_reward_count += 1
        #         htg_reward = 1
        #     if current_heading < 0 and self.previous_heading < 0:
        #         self.htg_penalty_count += 1
        #         htg_reward = 0

        # Waypoint reward
        if self.is_in_desired_position(self.position):
            rospy.loginfo("Reached waypoint position!!")

            # print("Robot position: " + str(self.position))
            # print("Desaired waypoint " + str(self.waypoint_desired_point))

            goal_waypoints = utils.get_local_goal_waypoints([self.position.x, self.position.y],
                                                            [self.original_desired_point.x,
                                                             self.original_desired_point.y], 
                                                             boundary_radius=0.3, epsilon=0.2) #original is 0.3
            self.waypoint_desired_point.x = goal_waypoints[0]
            self.waypoint_desired_point.y = goal_waypoints[1]
            waypoint_reward = 30 # original 200
            # print("Change desired point")
            # print(self.waypoint_desired_point)

            # Check if waypoint is within the goal point
            if self.is_in_true_desired_position(self.waypoint_desired_point):
                self.waypoint_desired_point.x = self.original_desired_point.x
                self.waypoint_desired_point.y = self.original_desired_point.y
                # print("Change desired point to actual goal point since it is near")
                # print(self.waypoint_desired_point)

        non_terminating_reward = step_reward + dtg_reward
        # non_terminating_reward = step_reward + dtg_reward + htg_reward + waypoint_reward + penalty_loop  + action_reward
        # non_terminating_reward = step_reward + dtg_reward + htg_reward + penalty_loop  + action_reward
        self.step_reward_count += 1

        if self.last_action is not None:
            reward = non_terminating_reward

        self.previous_distance = current_distance
        self.previous_heading = current_heading

        if done:
            print("step penalty count: ", str(self.step_reward_count))
            print("dtg reward count: ", str(self.dtg_reward_count))
            print("dtg penalty count: ", str(self.dtg_penalty_count))
            print("htg reward count: ", str(self.htg_reward_count))
            print("htg penalty count: ", str(self.htg_penalty_count))
            print("forward action reward count: ", str(self.forward_action_reward_count))
            print("left action reward count: ", str(self.left_turn_action_reward_count))
            print("right action reward count: ", str(self.right_turn_action_reward_count))
            print("stop action reward count: ", str(self.stop_action_reward_count))
            print("social nav reward count: ", str(self.social_nav_reward_count))
            print("----------------------------")
            if self.is_in_true_desired_position(self.position):
                rospy.loginfo("Reached goal position!!")
                self.episode_failure = False
                self.episode_success = True
                goal_reward = 100
                reward = goal_reward + non_terminating_reward
            else:
                rospy.loginfo("Collision!!")
                self.episode_failure = True
                self.episode_success = False
                collision_reward = -100
                reward = collision_reward + non_terminating_reward
            self.pub_cmd_vel.publish(Twist())

        return reward, done

    def step(self, action, step_counter, mode="discrete"):
        if mode == "discrete":
            if action == 0:  # FORWARD
                linear_speed = self.linear_forward_speed
                angular_speed = 0.0
                self.last_action = "FORWARDS"
            elif action == 1:  # LEFT
                linear_speed = self.linear_turn_speed
                angular_speed = self.angular_speed
                self.last_action = "TURN_LEFT"
            elif action == 2:  # RIGHT
                linear_speed = self.linear_turn_speed
                angular_speed = -1 * self.angular_speed
                self.last_action = "TURN_RIGHT"
        else:
            linear_speed = action[0]
            angular_speed = action[1]
            if linear_speed >= 0 and (((1.0 / 16.0) * -2.0) <= angular_speed <= (1.0 / 16.0) * 2.0):
                self.last_action = "FORWARD"
            elif linear_speed >= 0 and angular_speed > 0:
                self.last_action = "TURN_LEFT"
            elif linear_speed >= 0 and angular_speed < 0:
                self.last_action = "TURN_RIGHT"
            elif linear_speed == 0 and angular_speed == 0:
                self.last_action = "STOP"

        vel_cmd = Twist()
        vel_cmd.linear.x = linear_speed
        vel_cmd.angular.z = angular_speed
        self.vel_cmd = vel_cmd

        if self.vel_t0 == -1:  # Reset when deque is full
            self.vel_t0 = time.time()  # Start timer to get the timelapse between two positions of agent

        # Execute the actions to move the robot for 1 timestep
        start_timestep = time.time()
        self.pub_cmd_vel.publish(vel_cmd)
        time.sleep(0.15)
        end_timestep = time.time() - start_timestep
        if end_timestep < 0.05:
            time.sleep(0.05 - end_timestep)
            end_timestep += 0.05 - end_timestep + 0.1  # Without 0.1, the velocity is doubled
        # Get agent's position in a queue list. This is for collision cone implementation.
        self.agent_pose_deque.append([round(self.position.x, 3), round(self.position.y, 3)])
        self.agent_vel_timestep = end_timestep
        self.timestep_counter -= 1

        # Update previous robot yaw, to check for heading changes, for RVIZ tracking visualization
        self.previous_yaw = self.robot_yaw
        data_laser = None
        data_bumper = None
        data_cam = None
        while data_laser is None or data_bumper is None or data_cam is None:
            try:
                data_laser = rospy.wait_for_message('scan', LaserScan, timeout=5)
                data_bumper = utils.get_bumper_data()
                data_cam = rospy.wait_for_message('camera/depth/image_raw', Image, timeout=5)
                bridge = CvBridge()
                data_cam = bridge.imgmsg_to_cv2(data_cam, desired_encoding='passthrough')
            except:
                pass
        
        state, done = self.get_state(data_laser, data_bumper, data_cam, step_counter, action)
        reward, done = self.compute_reward(state, step_counter, done)

        return np.asarray(state), reward, done

    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            print("RESET PROXY")
            self.reset_proxy()
        except rospy.ServiceException as e:
            print("gazebo/reset_simulation service call failed")

        # Change obstacle positions
        self.state_msg_obs1.pose.position.x, self.state_msg_obs1.pose.position.y = self.obstacle_pose[self.obstacle_pose_count][0][0], self.obstacle_pose[self.obstacle_pose_count][0][1]
        self.state_msg_obs2.pose.position.x, self.state_msg_obs2.pose.position.y = self.obstacle_pose[self.obstacle_pose_count][1][0], self.obstacle_pose[self.obstacle_pose_count][1][1]
        self.state_msg_obs3.pose.position.x, self.state_msg_obs3.pose.position.y = self.obstacle_pose[self.obstacle_pose_count][2][0], self.obstacle_pose[self.obstacle_pose_count][2][1]
        self.obstacle_pose_count += 1
        if self.obstacle_pose_count == 9:
            self.obstacle_pose_count = 0

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp1 = set_state( self.state_msg_obs1 )
            resp2 = set_state( self.state_msg_obs2 )
            resp3 = set_state( self.state_msg_obs3 )
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        data_laser = None
        while data_laser is None:
            try:
                data_laser = rospy.wait_for_message('scan', LaserScan, timeout=5)
                data_bumper = utils.get_bumper_data()
                data_cam = rospy.wait_for_message('camera/depth/image_raw', Image, timeout=5)
                bridge = CvBridge()
                data_cam = bridge.imgmsg_to_cv2(data_cam, desired_encoding='passthrough')
            except:
                pass

        # Get random goal points
        # random_goal = np.random.randint(0,3)
        # self.original_desired_point.x = self.goal_points[random_goal][0]
        # self.original_desired_point.y = self.goal_points[random_goal][1]
        # self.original_desired_point.z = 0.0
        # print("Random goal point: ", self.original_desired_point)

        # Get initial heading and distance to goal
        self.previous_distance = self.get_distance_to_goal(self.position)
        self.previous_heading = self.get_heading_to_goal(self.position, self.orientation)
        self.previous_yaw = 3.14
        # print("data bumper: ", data_bumper)
        state, _ = self.get_state(data_laser, data_bumper, data_cam) 
        # print("RESET STATE: ", state)
        # Temporary (delete)
        self.step_reward_count = 0
        self.dtg_reward_count = 0
        self.htg_reward_count = 0
        self.dtg_penalty_count = 0
        self.htg_penalty_count = 0
        self.forward_action_reward_count = 0
        self.strong_right_turn_action_reward_count = 0
        self.strong_left_turn_action_reward_count = 0
        self.weak_right_turn_action_reward_count = 0
        self.weak_left_turn_action_reward_count = 0
        self.left_turn_action_reward_count = 0
        self.right_turn_action_reward_count = 0
        self.rotate_in_place_action_reward_count = 0
        self.social_safety_violation_count = 0
        self.ego_safety_violation_count = 0
        self.obstacle_present_step_counts = 0
        return np.asarray(state)

    def get_episode_status(self):

        return self.episode_success, self.episode_failure

    def get_social_safety_violation_status(self, step):
        # Obstacle present step counts means the total steps where an obstacle is detected within the robot's FOV
        # Otherwise, "step" just takes in all total number of steps regardless if it sees an obstacle or not
        # social_safety_score = 1.0 - ((self.social_safety_violation_count * 1.0) / self.obstacle_present_step_counts) #original
        social_safety_score = 1.0 - ((self.social_safety_violation_count * 1.0) / 9999) #modified
        # social_safety_score = 1.0 - ((self.social_safety_violation_count * 1.0) / step)

        return social_safety_score

    def get_ego_safety_violation_status(self, step):
        # Obstacle present step counts means the total steps where an obstacle is detected within the robot's FOV
        # Otherwise, "step" just takes in all total number of steps regardless if it sees an obstacle or not
        # ego_safety_score = 1.0 - ((self.ego_safety_violation_count * 1.0) / self.obstacle_present_step_counts)
        ego_safety_score = 1.0 - ((self.ego_safety_violation_count * 1.0) / 9999)
        # ego_safety_score = 1.0 - ((self.ego_safety_violation_count * 1.0) / step)

        return ego_safety_score

    def is_in_desired_position(self, current_position, epsilon=0.20):  # originally 0.05, changed to 0.20
        is_in_desired_pos = False

        x_pos_plus = self.waypoint_desired_point.x + epsilon
        x_pos_minus = self.waypoint_desired_point.x - epsilon
        y_pos_plus = self.waypoint_desired_point.y + epsilon
        y_pos_minus = self.waypoint_desired_point.y - epsilon

        x_current = current_position.x
        y_current = current_position.y

        x_pos_are_close = (x_current <= x_pos_plus) and (x_current > x_pos_minus)
        y_pos_are_close = (y_current <= y_pos_plus) and (y_current > y_pos_minus)

        is_in_desired_pos = x_pos_are_close and y_pos_are_close

        return is_in_desired_pos

    def is_in_true_desired_position(self, current_position, epsilon=0.20):  # originally 0.05, changed to 0.20
        is_in_desired_pos = False

        x_pos_plus = self.original_desired_point.x + epsilon
        x_pos_minus = self.original_desired_point.x - epsilon
        y_pos_plus = self.original_desired_point.y + epsilon
        y_pos_minus = self.original_desired_point.y - epsilon

        x_current = current_position.x
        y_current = current_position.y

        x_pos_are_close = (x_current <= x_pos_plus) and (x_current > x_pos_minus)
        y_pos_are_close = (y_current <= y_pos_plus) and (y_current > y_pos_minus)

        is_in_desired_pos = x_pos_are_close and y_pos_are_close

        return is_in_desired_pos
