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
import utils_real as utils

import json
from PIL import Image
from io import BytesIO
import cv2


from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

import requests
import pyrealsense2 as rs

class Env:
    def __init__(self, action_dim=2, max_step=200):
        self.odom = requests.get('http://192.168.0.101/data/odometry')
        self.odom = self.odom.json()
        self.odom[0] = (self.odom[0] - 1) * -1
        self.odom[1] = (self.odom[1] + 1) * -1
        self.prev_pos = self.odom
        

        self.action_dim = action_dim
        # # Keys CTRL + c will stop script
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
        # self.starting_point.x = rospy.get_param("/robotino/starting_pose/x")
        # self.starting_point.y = rospy.get_param("/robotino/starting_pose/y")
        # self.starting_point.z = rospy.get_param("/robotino/starting_pose/z")
        self.starting_point.x, self.starting_point.y, self.rot, self.vx, self.vy, self.omega, self.sec = 0.75, -0.75, 0, 0, 0, 0, 0
        self.starting_point.z = rospy.get_param("/robotino/starting_pose/z")

        self.original_desired_point = Point()
        self.original_desired_point.x = rospy.get_param("/robotino/desired_pose/x") 
        self.original_desired_point.y = rospy.get_param("/robotino/desired_pose/y") 
        self.original_desired_point.z = rospy.get_param("/robotino/desired_pose/z")

        # self.goal_points = [[-1,-1],[-1,0],[-1,1]]

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
        self.ego_penalty_count = 0
        self.last_action = "FORWARD"

        self.total_x_travelled = 0
        self.total_y_travelled = 0

        # RVIZ visualization markers to see Collision Probabilities of obstacles w.r.t robot's motion
        # self.pub_obs1_pose_text = rospy.Publisher('/obstacle_text_poses/1', Marker, queue_size=1)
        # self.pub_obs2_pose_text = rospy.Publisher('/obstacle_text_poses/2', Marker, queue_size=1)
        # self.pub_obs3_pose_text = rospy.Publisher('/obstacle_text_poses/3', Marker, queue_size=1)

        # self.pub_obs1_pose_shape = rospy.Publisher('/obstacle_text_shape/1', Marker, queue_size=1)
        # self.pub_obs2_pose_shape = rospy.Publisher('/obstacle_text_shape/2', Marker, queue_size=1)
        # self.pub_obs3_pose_shape = rospy.Publisher('/obstacle_text_shape/3', Marker, queue_size=1)

        # for testing
        self.inc = 1
        
        self.collision_count = 0
        self.collision_penalty = 0

        self.pipeline = rs.pipeline()

        # Configure streams
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Start streaming
        self.pipeline.start(self.config)

        self.start_time = time.time()

    def shutdown(self):
        rospy.loginfo("Stopping TurtleBot")
        # self.pub_cmd_vel.publish(Twist())
        requests.post('http://192.168.0.101/data/omnidrive', json=[0,0,0])
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
        a = np.array((pstart[0], pstart[1]))
        b = np.array((p_end.x, p_end.y))
        # print("a: ", a)
        # print("b: ", b)

        distance = np.linalg.norm(a - b)

        # print("Distance: ", distance)

        return distance

    def get_distance_to_goal(self, current_position):
        distance = self.get_distance_from_point(current_position,
                                                self.waypoint_desired_point)

        return distance

    def get_actual_distance_to_goal(self, current_position):
        # print("Original desired point: ", self.original_desired_point)
        distance = self.get_distance_from_point(current_position,
                                                self.original_desired_point)

        return distance

    def get_actual_heading_to_goal(self, current_position):
        current_pos_x = current_position[0] + self.starting_point.x
        current_pos_y = current_position[1] + self.starting_point.y

        # yaw = self.get_angle_from_point(current_orientation)
        yaw = current_position[2]
        goal_angle = math.atan2(self.original_desired_point.y - current_pos_y,
                                self.original_desired_point.x - current_pos_x)

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        return heading

    def get_state(self, data_laser, data_bumper, data_cam, data_odom, step_counter=0):

        # distance_to_goal = round(self.get_distance_to_goal(data_odom), 2)
        # heading_to_goal = round(self.get_heading_to_goal(self.position, self.orientation), 2)
        actual_heading_to_goal = round(self.get_actual_heading_to_goal(data_odom), 2)
        actual_distance_to_goal = round(self.get_actual_distance_to_goal(data_odom), 2)

        if not self.done:
            if data_bumper[0]:
                print("DONE: MINIMUM RANGE")
                # print("MINIMUM: ", str(min(current_scans)))
                self.done = True

            if self.is_in_true_desired_position(data_odom):
                print("DONE: IN DESIRED POSITION")
                self.done = True

            if step_counter >= self.max_steps:
                print("DONE: STEP COUNTER > MAX STEP")
                self.done = True

        agent_position = [round(data_odom[0], 3), round(data_odom[1], 3)]
        agent_orientation = [round(data_odom[2], 3)]
        agent_velocity = [round(data_odom[3], 3), round(data_odom[4], 3)]

        goal_heading_distance = [actual_heading_to_goal, actual_distance_to_goal]

        # Image processing
        cnn_result = utils.cnn(data_cam)

        # For inserting goal point to the state
        desired_point = [self.original_desired_point.x*10, self.original_desired_point.y*10]

        state = (data_laser + goal_heading_distance + agent_position + agent_orientation + agent_velocity
                 + cnn_result + data_bumper + desired_point)

        # Round items in state to 2 decimal places
        state = list(np.around(np.array(state), 3))

        print("Scan ranges: ", data_laser)
        print("Goal heading distance: ", goal_heading_distance)
        print("Agent position: ", agent_position)
        print("Agent orientation: ", agent_orientation)
        print("Agent velocity: ", agent_velocity)
        # print("CNN result: ", cnn_result)
        print("Bumper data: ", data_bumper)
        print("Desired point: ", desired_point)

        return state, self.done

    def compute_reward(self, state, step_counter, done):
        current_heading = state[9]
        current_distance = state[10]

        penalty_loop = 0

        if (step_counter % 75) == 0:
            travel_x = self.prev_pos[0] - self.odom[0]
            travel_y = self.prev_pos[1] - self.odom[1]
            self.prev_pos[0] = self.odom[0]
            self.prev_pos[1] = self.odom[1]

            if math.sqrt((travel_x**2) + (travel_y**2)) < 0.4:
                rospy.loginfo("Robot is stuck in a loop!")
                penalty_loop = -100


        step_reward = -1  # step penalty

        # Ego penalty
        scan_ranges_temp = []
        for i in range(9):
            scan_ranges_temp.append(state[i])
        
        ego_penalty = 0
        if min(scan_ranges_temp) < 0.4:
            self.ego_penalty_count += 1
            ego_penalty = -2

        non_terminating_reward = step_reward + ego_penalty + penalty_loop
        self.step_reward_count += 1

        if self.last_action is not None:
            reward = non_terminating_reward

        self.previous_distance = current_distance
        self.previous_heading = current_heading

        if done:
            print("step penalty count: ", str(self.step_reward_count))
            print("ego penalty count: ", str(self.ego_penalty_count))
            print("----------------------------")
            if self.is_in_true_desired_position(self.odom):
                rospy.loginfo("Reached goal position!!")
                self.episode_failure = False
                self.episode_success = True
                goal_reward = 100
                time_lapse = time.time() - self.start_time
                reward_time = 200 - (time_lapse*2)
                reward = goal_reward + non_terminating_reward + round(reward_time)
            else:
                rospy.loginfo("Collision!!")
                self.episode_failure = True
                self.episode_success = False
                early_stop_penalty = 0
                if self.step_reward_count < 50:
                    early_stop_penalty = -75
                collision_reward = -100
                reward = collision_reward + non_terminating_reward + early_stop_penalty

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

        if self.vel_t0 == -1:  # Reset when deque is full
            self.vel_t0 = time.time()  # Start timer to get the timelapse between two positions of agent

        # Execute the actions to move the robot for 1 timestep
        start_timestep = time.time()
        # print("Linear speed: ", linear_speed)
        # print("Angular speed: ", angular_speed)
        # print("Yaw: ", self.odom[2])
        angular_speed = angular_speed * -1

        utils.post_omnidrive(linear_speed, angular_speed)
        
        time.sleep(0.15)
        end_timestep = time.time() - start_timestep
        if end_timestep < 0.05:
            time.sleep(0.05 - end_timestep)
            end_timestep += 0.05 - end_timestep + 0.1  # Without 0.1, the velocity is doubled
        
        self.agent_vel_timestep = end_timestep
        self.timestep_counter -= 1

        # Update previous robot yaw, to check for heading changes, for RVIZ tracking visualization
        self.previous_yaw = self.odom[2]
        data_laser = None
        data_bumper = None
        data_cam = None
        while data_laser is None or data_bumper is None or data_cam is None:
            try:
                data_laser = utils.get_laser()
                data_laser = data_laser.json()
                data_bumper = utils.get_bumper_data()
                frames = self.pipeline.wait_for_frames()
                depth = frames.get_depth_frame()
                decimation = rs.decimation_filter()
                # depth = decimation.process(depth)

                spatial = rs.spatial_filter()
                # depth = spatial.process(depth)
                # colorized_depth = np.asanyarray(colorizer.colorize(filtered_depth).get_data())
                # plt.imshow(colorized_depth)

                hole_filling = rs.hole_filling_filter(2)
                filled_depth = hole_filling.process(depth)
                if not depth: continue
                depth_image = np.asanyarray(filled_depth.get_data())
                # depth_image = np.asanyarray(depth.get_data())
                data_cam = depth_image * depth.get_units()
            except:
                pass
        
        data_odom = requests.get('http://192.168.0.101/data/odometry')
        data_odom = data_odom.json()
        data_odom[0] = (data_odom[0] - 1)*-1
        data_odom[1] = (data_odom[1] + 1)*-1
        if data_odom[2] > 0:
            data_odom[2] = data_odom[2] - pi
        else:
            data_odom[2] = data_odom[2] + pi
        self.odom = data_odom

        if data_bumper[0] == False:
            data_bumper[0] = 0
        else:
            data_bumper[0] = 1

        state, done = self.get_state(data_laser, data_bumper, data_cam, data_odom, step_counter)
        reward, done = self.compute_reward(state, step_counter, done)

        return np.asarray(state), reward, done

    def reset(self):

        # Reset variabel
        self.ego_penalty_count = 0

        self.start_time = time.time()

        data_laser = None
        data_bumper = None
        data_cam = None
        while data_laser is None and data_bumper is None and data_cam is None:
            try:
                data_laser = utils.get_laser()
                data_laser = data_laser.json()
                data_bumper = utils.get_bumper_data()
                frames = self.pipeline.wait_for_frames()
                depth = frames.get_depth_frame()
                decimation = rs.decimation_filter()
                # depth = decimation.process(depth)

                spatial = rs.spatial_filter()
                # depth = spatial.process(depth)
                # colorized_depth = np.asanyarray(colorizer.colorize(filtered_depth).get_data())
                # plt.imshow(colorized_depth)

                hole_filling = rs.hole_filling_filter(2)
                filled_depth = hole_filling.process(depth)
                if not depth: continue
                depth_image = np.asanyarray(filled_depth.get_data())
                # depth_image = np.asanyarray(depth.get_data())
                data_cam = depth_image * depth.get_units()
                # print(data_cam.shape)
            except:
                pass

        # Get initial heading and distance to goal
        data_odom = requests.get('http://192.168.0.101/data/odometry')
        data_odom = data_odom.json()
        data_odom[0] = (data_odom[0] - 1) * -1 # disesuaikan dengan koordinat gazebo
        data_odom[1] = (data_odom[1] + 1) * -1 
        if data_odom[2] > 0:
            data_odom[2] = data_odom[2] - pi
        else:
            data_odom[2] = data_odom[2] + pi
        
        self.odom = data_odom
        if data_bumper[0] == False:
            data_bumper[0] = 0
        else:
            data_bumper[0] = 1
        self.previous_distance = self.get_actual_distance_to_goal(data_odom)
        self.previous_heading = self.get_actual_heading_to_goal(data_odom)
        self.previous_yaw = 3.14
        state, _ = self.get_state(data_laser, data_bumper, data_cam, data_odom) 
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

        x_current = current_position[0]
        y_current = current_position[1]

        x_pos_are_close = (x_current <= x_pos_plus) and (x_current > x_pos_minus)
        y_pos_are_close = (y_current <= y_pos_plus) and (y_current > y_pos_minus)

        is_in_desired_pos = x_pos_are_close and y_pos_are_close

        return is_in_desired_pos
