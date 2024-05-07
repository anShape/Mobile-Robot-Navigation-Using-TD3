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
import rospy


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
        self.starting_point.x = rospy.get_param("/turtlebot3/starting_pose/x")
        self.starting_point.y = rospy.get_param("/turtlebot3/starting_pose/y")
        self.starting_point.z = rospy.get_param("/turtlebot3/starting_pose/z")

        self.original_desired_point = Point()
        self.original_desired_point.x = rospy.get_param("/turtlebot3/desired_pose/x")
        self.original_desired_point.y = rospy.get_param("/turtlebot3/desired_pose/y")
        self.original_desired_point.z = rospy.get_param("/turtlebot3/desired_pose/z")

        self.waypoint_desired_point = Point()
        self.waypoint_desired_point.x = self.original_desired_point.x
        self.waypoint_desired_point.y = self.original_desired_point.y
        self.waypoint_desired_point.z = self.original_desired_point.z

        self.linear_forward_speed = rospy.get_param('/turtlebot3/linear_forward_speed')
        self.linear_turn_speed = rospy.get_param('/turtlebot3/linear_turn_speed')
        self.angular_speed = rospy.get_param('/turtlebot3/angular_speed')
        self.scan_ranges = rospy.get_param('/turtlebot3/scan_ranges')
        self.max_scan_range = rospy.get_param('/turtlebot3/max_scan_range')
        self.min_scan_range = rospy.get_param('/turtlebot3/min_scan_range')
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
            print("ini step counter 1")
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

        #INI GA DIPAKE
        
        # obstacle_poses = utils.convert_laserscan_to_coordinate(scan_range, self.scan_ranges, self.position, yaw, 360) 

        # # Get average distance between laserscans based on ground truth scans, for hungarian association
        # # agent_prev_x, agent_prev_y = 0, 0
        # if step_counter == 0:
        #     self.ground_truth_poses = utils.convert_laserscan_to_coordinate(self.ground_truth_scans, self.scan_ranges,
        #                                                                     self.position, yaw, 360)
        #     self.bounding_box_size = utils.compute_average_bounding_box_size(self.ground_truth_poses)
        #     self.timestep_delay = time.time()

        #     # Get agent's position in a queue list. This is for collision cone implementation.
        #     self.agent_pose_deque.append([round(self.position.x, 3), round(self.position.y, 3)])

        # # Get obstacle region and position
        # obstacle_region = []
        # for i in range(len(obstacle_poses)):
        #     # Get obstacles' region. This is for social navigation implementation.
        #     obs_heading = utils.get_heading_to_obs(self.position, self.orientation, obstacle_poses[i])
        #     region = utils.get_obstacle_region(self.position, yaw, obstacle_poses[i], scan_range[i], obs_heading)
        #     obstacle_region.append(region)

        #     # Get obstacles' position in a queue list. This is for collision cone.
        #     self.obstacle_pose_deque[i].append(obstacle_poses[i])
            

        # # Check if scans are occupied by an obstacle or not (free space)
        # current_scans = scan_range
        # current_scans_is_gt = []
        # for i in range(len(current_scans)):
        #     if 1.0 * self.ground_truth_scans[i] <= current_scans[i] <= 1.0 * self.ground_truth_scans[i]:
        #         current_scans_is_gt.append(True)
        #     else:
        #         current_scans_is_gt.append(False)
                

        # Replace poses with None when scans are ground truth (a.k.a free space)
        # filtered_front_obstacle_poses = []
        # filtered_scan_ranges = []

        # for i in range(len(obstacle_poses)):
        #     if current_scans_is_gt[i] is True:
        #         # pass
        #         filtered_front_obstacle_poses.append(obstacle_poses[i])
        #         filtered_scan_ranges.append(round(scan_range[i], 3))
        #     else:
        #         filtered_front_obstacle_poses.append(obstacle_poses[i])
        #         filtered_scan_ranges.append(round(scan_range[i], 3))

        # current_grads = []
        # for i in range(len(filtered_front_obstacle_poses)):
        #     if filtered_scan_ranges[i] == 0.6:
        #         current_grads.append(None)
        #     else:
        #         if i == (len(filtered_front_obstacle_poses) - 1):
        #             if (filtered_front_obstacle_poses[i][1] - filtered_front_obstacle_poses[0][1]) == 0:
        #                 gradient = 0.0
        #             else:
        #                 gradient = (filtered_front_obstacle_poses[i][0] - filtered_front_obstacle_poses[0][0]) \
        #                            / (filtered_front_obstacle_poses[i][1] - filtered_front_obstacle_poses[0][1])
        #         else:
        #             if (filtered_front_obstacle_poses[i][1] - filtered_front_obstacle_poses[i + 1][1]) == 0:
        #                 gradient = 0.0
        #             else:
        #                 gradient = (filtered_front_obstacle_poses[i][0] - filtered_front_obstacle_poses[i + 1][0]) \
        #                            / (filtered_front_obstacle_poses[i][1] - filtered_front_obstacle_poses[i + 1][1])
        #         current_grads.append(round(gradient, 3))

        # change_grads = [None] * len(current_grads)
        # last_grad = None
        # for i in range(len(current_grads)):
        #     if current_grads[i] is None:
        #         change_grads[i] = None
        #     else:
        #         if len(current_grads) == 1:
        #             change_grads[i] = last_grad
        #         elif i == len(current_grads) - 1:
        #             # _change_m = abs(ms[i] - ms[0])
        #             change_grads[i] = last_grad
        #         else:
        #             if current_grads[i + 1] is not None:
        #                 _change_m = abs(current_grads[i] - current_grads[i + 1])
        #                 last_grad = _change_m
        #                 change_grads[i] = _change_m
        #             else:
        #                 _change_m = None
        #                 last_grad = _change_m
        #                 change_grads[i] = _change_m

        # Find which scan is a wall or obstacle object (object type differentiation)
        # An item in scans_object_type contains [object type, range] for estimating number of scans an obstacle contains
        # given the range
        # _scans_object_type = [None] * len(change_grads)

        # last_type = None
        # du_count = 0  # Delayed update count
        # for i in range(len(filtered_front_obstacle_poses)):
        #     if change_grads[i] is None:
        #         pass
        #     else:
        #         if i == len(change_grads) - 1:
        #             pass
        #         # Is the first object a wall type or an obstacle type?
        #         elif change_grads[i] == 0:
        #             _scans_object_type[i] = ["w", round(filtered_scan_ranges[i], 3), filtered_front_obstacle_poses[i]]
        #             last_type = _scans_object_type[i]
        #         else:
        #             _scans_object_type[i] = ["o", round(filtered_scan_ranges[i], 3), filtered_front_obstacle_poses[i]]
        #             if du_count != 1:
        #                 if change_grads[i + 1] == 0:
        #                     _scans_object_type[i] = ["w", round(filtered_scan_ranges[i], 3),
        #                                              filtered_front_obstacle_poses[i]]
        #                     last_type = _scans_object_type[i]
        #                     du_count = 0
        #                 if change_grads[i] is not None and change_grads[i + 1] is None:
        #                     pass
        #                 else:
        #                     if abs(change_grads[i] - change_grads[i + 1]) == 0:
        #                         _scans_object_type[i] = ["w", round(filtered_scan_ranges[i], 3),
        #                                                  filtered_front_obstacle_poses[i]]
        #                         last_type = _scans_object_type[i]
        #                         du_count = 0
        #                     else:
        #                         _scans_object_type[i] = last_type
        #                         du_count += 1
        #             else:
        #                 _scans_object_type[i] = ["o", round(filtered_scan_ranges[i], 3),
        #                                          filtered_front_obstacle_poses[i]]
        #                 last_type = _scans_object_type[i]
        #                 if change_grads[i + 1] == 0:
        #                    du_count = 0

        # Group scans in between None object types (ground truth scans are True)
        # e.g [None, ['o', 1.234], ['o', 1.234], None, ['o', 1.234]]
        # = [[['o', 1.234], ['o', 1.234]], [['o', 1.234]]] <--- 2 objects present
        # scans_object_type = []
        # _scan_group = []
        # for i in range(len(_scans_object_type)):
        #     if i == len(_scans_object_type) - 1:
        #         pass
        #     else:
        #         if _scans_object_type[i] is None:
        #             _scan_group.append(_scans_object_type[i])
        #         elif _scans_object_type[i] is not None and _scans_object_type[i + 1] is not None:
        #             _scan_group.append(_scans_object_type[i])
        #         else:
        #             _scan_group.append(_scans_object_type[i])
        #             scans_object_type.append(_scan_group)
        #             _scan_group = []

        # Object type estimate -> Gradient method
        # Object segmentation/count estimate -> Hungarian association algorithm
        # Object type further confirmation -> distance to num of scans proportionality method
        # estimated_scans_distances = []
        # estimated_scans_object_types = []
        # estimated_scans_object_poses = []
        # for i in range(len(_scans_object_type)):
        #     _closest_scans, _obj_types, _obj_poses = [], [], []
        #     if _scans_object_type[i] is None:
        #         estimated_scans_object_types.append('none')
        #         estimated_scans_distances.append(filtered_scan_ranges[i])
        #         estimated_scans_object_poses.append(obstacle_poses[i])
        #     else:
        #         estimated_scans_object_types.append(_scans_object_type[i][0])
        #         estimated_scans_distances.append(_scans_object_type[i][1])
        #         estimated_scans_object_poses.append(_scans_object_type[i][2])

        # Object segmentation/count estimate (Hungarian association)
        # iou, _iou = [], []
        # _segmented_scan_object_types, _segmented_type = [], []
        # _segmented_scan_object_distances, _segmented_dist = [], []
        # _segmented_scan_object_poses, _segmented_poses = [], []
        # for i in range(len(estimated_scans_object_types)):
        #     if i == len(estimated_scans_object_types) - 1:
        #         if utils.is_associated(estimated_scans_object_poses[i], estimated_scans_object_poses[0],
        #                                self.bounding_box_size * 2) is True:  # Twice because of the blindspot
        #             _segmented_type.append(estimated_scans_object_types[i])
        #             _segmented_dist.append(estimated_scans_distances[i])
        #             _segmented_poses.append(estimated_scans_object_poses[i])
        #             _segmented_scan_object_types.append(_segmented_type)
        #             _segmented_scan_object_distances.append(_segmented_dist)
        #             _segmented_scan_object_poses.append(_segmented_poses)
        #             _segmented_type, _segmented_dist, _segmented_poses = [], [], []
        #         else:
        #             _segmented_type.append(estimated_scans_object_types[i])
        #             _segmented_dist.append(estimated_scans_distances[i])
        #             _segmented_poses.append(estimated_scans_object_poses[i])
        #             _segmented_scan_object_types.append(_segmented_type)
        #             _segmented_scan_object_distances.append(_segmented_dist)
        #             _segmented_scan_object_poses.append(_segmented_poses)
        #             _segmented_type, _segmented_dist, _segmented_poses = [], [], []
        #     else:
        #         if utils.is_associated(estimated_scans_object_poses[i], estimated_scans_object_poses[i + 1],
        #                                self.bounding_box_size) is True:
        #             _segmented_type.append(estimated_scans_object_types[i])
        #             _segmented_dist.append(estimated_scans_distances[i])
        #             _segmented_poses.append(estimated_scans_object_poses[i])

        #         else:
        #             _segmented_type.append(estimated_scans_object_types[i])
        #             _segmented_dist.append(estimated_scans_distances[i])
        #             _segmented_poses.append(estimated_scans_object_poses[i])
        #             _segmented_scan_object_types.append(_segmented_type)
        #             _segmented_scan_object_distances.append(_segmented_dist)
        #             _segmented_scan_object_poses.append(_segmented_poses)
        #             _segmented_type, _segmented_dist, _segmented_poses = [], [], []

        # Fix issue of detecting two seperate objects between the first scan and the last scan
        # There is a small region where no raycast is present there. This fix uses Hungarian association score of about
        # twice the average bounding box size since the small region occupies about one bounding box size.
        # for i in range(1):
        #     if len(_segmented_scan_object_types) > 1:
        #         if utils.is_associated(_segmented_scan_object_poses[0][0], _segmented_scan_object_poses[-1][-1],
        #                                self.bounding_box_size * 2) is True:
        #             _concat_scan_type = _segmented_scan_object_types[0] + _segmented_scan_object_types[-1]
        #             _concat_scan_dist = _segmented_scan_object_distances[0] + _segmented_scan_object_distances[-1]
        #             _concat_scan_pose = _segmented_scan_object_poses[0] + _segmented_scan_object_poses[-1]
        #             _segmented_scan_object_types[0] = _concat_scan_type
        #             _segmented_scan_object_distances[0] = _concat_scan_dist
        #             _segmented_scan_object_poses[0] = _concat_scan_pose
        #             _segmented_scan_object_types.pop(-1)
        #             _segmented_scan_object_distances.pop(-1)
        #             _segmented_scan_object_poses.pop(-1)

        # segmented_scan_object_types = _segmented_scan_object_types
        # segmented_scan_object_distances = _segmented_scan_object_distances
        # segmented_scan_object_poses = _segmented_scan_object_poses

        # Fix wall scans being grouped with None scans
        # __segmented_scan_object_types, __segmented_type = [], []
        # __segmented_scan_object_distances, __segmented_dist = [], []
        # __segmented_scan_object_poses, __segmented_poses = [], []
        # for i in range(len(segmented_scan_object_distances)):
        #     if not any(a != 0.6 for a in segmented_scan_object_distances[i]):
        #         continue
        #     else:
        #         for j in range(len(segmented_scan_object_types[i])):
        #             if j == len(segmented_scan_object_types[i]) - 1:
        #                 __segmented_type.append(segmented_scan_object_types[i][j])
        #                 __segmented_dist.append(segmented_scan_object_distances[i][j])
        #                 __segmented_poses.append(segmented_scan_object_poses[i][j])
        #                 __segmented_scan_object_types.append(__segmented_type)
        #                 __segmented_scan_object_distances.append(__segmented_dist)
        #                 __segmented_scan_object_poses.append(__segmented_poses)
        #                 __segmented_type, __segmented_dist, __segmented_poses = [], [], []
        #             else:
        #                 if segmented_scan_object_distances[i][j] == 0.6 and \
        #                         segmented_scan_object_distances[i][j + 1] == 0.6:
        #                     __segmented_type.append(segmented_scan_object_types[i][j])
        #                     __segmented_dist.append(segmented_scan_object_distances[i][j])
        #                     __segmented_poses.append(segmented_scan_object_poses[i][j])
        #                 elif segmented_scan_object_distances[i][j] != 0.6 and \
        #                         segmented_scan_object_distances[i][j + 1] != 0.6:
        #                     __segmented_type.append(segmented_scan_object_types[i][j])
        #                     __segmented_dist.append(segmented_scan_object_distances[i][j])
        #                     __segmented_poses.append(segmented_scan_object_poses[i][j])
        #                 else:
        #                     __segmented_type.append(segmented_scan_object_types[i][j])
        #                     __segmented_dist.append(segmented_scan_object_distances[i][j])
        #                     __segmented_poses.append(segmented_scan_object_poses[i][j])
        #                     __segmented_scan_object_types.append(__segmented_type)
        #                     __segmented_scan_object_distances.append(__segmented_dist)
        #                     __segmented_scan_object_poses.append(__segmented_poses)
        #                     __segmented_type, __segmented_dist, __segmented_poses = [], [], []

        #         segmented_scan_object_types[i] = __segmented_scan_object_types
        #         segmented_scan_object_distances[i] = __segmented_scan_object_distances
        #         segmented_scan_object_poses[i] = __segmented_scan_object_poses
        #         __segmented_scan_object_types = []
        #         __segmented_scan_object_distances = []
        #         __segmented_scan_object_poses = []

        # segmented_scan_object_distances_2d = []
        # segmented_scan_object_types_2d = []
        # segmented_scan_object_poses_2d = []

        # # Temporary fix: remove nones from first and last index of a list
        # for i in range(len(segmented_scan_object_distances)):
        #     if any(isinstance(item, list) for item in segmented_scan_object_distances[i]):
        #         for j in range(len(segmented_scan_object_distances[i])):
        #             segmented_scan_object_distances_2d.append(segmented_scan_object_distances[i][j])
        #             segmented_scan_object_types_2d.append(segmented_scan_object_types[i][j])
        #             segmented_scan_object_poses_2d.append(segmented_scan_object_poses[i][j])
        #     else:
        #         segmented_scan_object_distances_2d.append(segmented_scan_object_distances[i])
        #         segmented_scan_object_types_2d.append(segmented_scan_object_types[i])
        #         segmented_scan_object_poses_2d.append(segmented_scan_object_poses[i])

        # Object type confirmation (Scan distance to number of obstacle type estimate with proportionality)
        # confirmed_scan_object = []
        # for i in range(len(segmented_scan_object_types_2d)):
        #     if not any(a != 0.6 for a in segmented_scan_object_distances_2d[i]):
        #         continue
        #     elif len(segmented_scan_object_distances_2d[i]) < 4:
        #         for j in range(len(segmented_scan_object_distances_2d[i])):
        #             segmented_scan_object_distances_2d[i][j] = 0.6
        #     else:
        #         _center_item = len(segmented_scan_object_types_2d[i]) / 2
        #         _center_item = int(_center_item)
        #         estimated_obstacle_count = utils.estimate_num_obs_scans(
        #             segmented_scan_object_distances_2d[i][_center_item],
        #             self.max_scan_range,
        #             self.min_scan_range)
        #         current_obstacle_count = segmented_scan_object_types_2d[i].count('o')
        #         confirmed_obstacle_score = float(current_obstacle_count) / min(len(segmented_scan_object_types_2d[i]),
        #                                                                        estimated_obstacle_count)

        #         if len(set(segmented_scan_object_types_2d[i])) > 1:  # Not all identical
        #             if confirmed_obstacle_score >= 0.5:
        #                 if segmented_scan_object_types_2d[i].count('o') > segmented_scan_object_types_2d[i].count('w'):
        #                     confirmed_scan_object.append(['o', segmented_scan_object_poses_2d[i][_center_item],
        #                                                   segmented_scan_object_distances_2d[i][_center_item]])
        #                 else:
        #                     confirmed_scan_object.append(['w', segmented_scan_object_poses_2d[i][_center_item],
        #                                                   segmented_scan_object_distances_2d[i][_center_item]])
        #             else:
        #                 if len(segmented_scan_object_types_2d[i]) <= estimated_obstacle_count:
        #                     if segmented_scan_object_types_2d[i].count('o') > segmented_scan_object_types_2d[i].count(
        #                             'w'):
        #                         confirmed_scan_object.append(['o', segmented_scan_object_poses_2d[i][_center_item],
        #                                                       segmented_scan_object_distances_2d[i][_center_item]])
        #                     else:
        #                         confirmed_scan_object.append(['w', segmented_scan_object_poses_2d[i][_center_item],
        #                                                       segmented_scan_object_distances_2d[i][_center_item]])
        #                 else:
        #                     confirmed_scan_object.append(['w', segmented_scan_object_poses_2d[i][_center_item],
        #                                                   segmented_scan_object_distances_2d[i][_center_item]])
        #         else:  # identical
        #             if 'w' in segmented_scan_object_types_2d[i]:
        #                 if len(segmented_scan_object_types_2d[i]) <= min(len(segmented_scan_object_types_2d),
        #                                                                  estimated_obstacle_count):
        #                     pass
        #                 else:
        #                     confirmed_scan_object.append(['w', segmented_scan_object_poses_2d[i][_center_item],
        #                                                   segmented_scan_object_distances_2d[i][_center_item]])
        #             else:
        #                 if len(segmented_scan_object_types_2d[i]) <= min(len(segmented_scan_object_types_2d),
        #                                                                  estimated_obstacle_count):
        #                     pass
        #                 else:
        #                     confirmed_scan_object.append(['o', segmented_scan_object_poses_2d[i][_center_item],
        #                                                   segmented_scan_object_distances_2d[i][_center_item]])

        # Fix segmented scans from message dropping out in
        # ___segmented_scan_object_types, ___segmented_type = [], []
        # ___segmented_scan_object_distances, ___segmented_dist = [], []
        # ___segmented_scan_object_poses, ___segmented_poses = [], []
        # for i in range(len(segmented_scan_object_distances_2d)):
        #     if 0.6 in segmented_scan_object_distances_2d[i]:
        #         ___segmented_dist += segmented_scan_object_distances_2d[i]
        #         if i == len(segmented_scan_object_distances_2d) - 1:
        #             ___segmented_scan_object_distances.append(___segmented_dist)
        #             ___segmented_dist = []
        #     else:
        #         ___segmented_scan_object_distances.append(___segmented_dist)
        #         ___segmented_scan_object_distances.append(segmented_scan_object_distances_2d[i])
        #         ___segmented_dist = []

        # # Get wall scans and obstacle object scans
        # wall_scans, obstacle_scans = [], []
        # for i in range(len(confirmed_scan_object)):
        #     if confirmed_scan_object[i][0] == "w":
        #         wall_scans.append(confirmed_scan_object[i])
        #     if confirmed_scan_object[i][0] == "o":
        #         obstacle_scans.append(confirmed_scan_object[i])

        # print("CONFIRMED SCAN: ", confirmed_scan_object)
        # print("OBSTACLE SCANS: ", obstacle_scans)
        # print("OBSTACLE COUNT: ", len(obstacle_scans))
        # print("WALL COUNT: ", len(wall_scans))
        # print("WALL SCANS: ", wall_scans)

        # Check if there is obstacle: for calculating accurate ego and social score (don't count all steps but just
        # steps where an obstacle is "seen" by the robot
        # if len(obstacle_scans) > 0:
        #     self.obstacle_present_step_counts += 1

        # iou, _iou = [], []
        # checked_obj_scans = None
        # tracked_obstacles_copy = self.tracked_obstacles.copy()
        # tracked_obstacles_key_copy = self.tracked_obstacles_keys[:]
        # if len(tracked_obstacles_copy) == 0:
        #     for i in range(len(obstacle_scans)):
        #         unique_id = uuid4()
        #         # Get tracked obstacles in the following format [<object type>, <coord list>, <distance>, ...<append>]
        #         _tracked_obs = obstacle_scans[i][:]
        #         _tracked_obs.append(deque([obstacle_scans[i][1]]))
        #         _tracked_obs.append(time.time())
        #         _tracked_obs.append(-1)  # velocity
        #         _tracked_obs.append([0.0, 0.0])  # velocity X and Y
        #         # Assign a unique id to tracked obstacles
        #         self.tracked_obstacles[unique_id] = _tracked_obs
        #     self.tracked_obstacles_keys = list(self.tracked_obstacles.keys())
        #     tracked_obstacles_key_copy = self.tracked_obstacles_keys[:]
        # else:
        #     # Check if object is associated with IOU score
        #     val, idx = None, None
        #     for i in range(len(tracked_obstacles_copy)):
        #         # Remove obstacle pose from deque list if greater than 1
        #         if tracked_obstacles_key_copy[i] in self.tracked_obstacles:
        #             if len(tracked_obstacles_copy[tracked_obstacles_key_copy[i]][3]) > 1:
        #                 self.tracked_obstacles[tracked_obstacles_key_copy[i]][3].popleft()

        #         _tracked_obs = list(tracked_obstacles_copy.get(tracked_obstacles_key_copy[i]))[1]
        #         if len(confirmed_scan_object) == 0:
        #             # Remove all "tracked" obstacle from tracking list
        #             self.tracked_obstacles.pop(tracked_obstacles_key_copy[i - 1])
        #             self.tracked_obstacles_keys.remove(tracked_obstacles_key_copy[i])
        #         else:
        #             for j in range(len(list(confirmed_scan_object))):
        #                 _iou.append(utils.get_iou(_tracked_obs, confirmed_scan_object[j][1], 0.0505))
        #                 val, idx = max((val, idx) for (idx, val) in enumerate(_iou))  # max iou value and index
        #         iou.append(_iou)
        #         _iou = []
        #         checked_obj_scans = [False] * len(iou[0])

        # iou_copy = iou[:]
        # iou2, _iou2 = [], []
        # reupdate_tracking = False

        # for i in range(len(iou_copy)):
        #     if iou_copy[i]:
        #         max_value_idx = iou_copy[i].index(max(iou_copy[i]))
        #         if max(iou_copy[i]) > 0.0:
        #             tracked_obstacles_copy[tracked_obstacles_key_copy[i]][1] = confirmed_scan_object[max_value_idx][1]
        #             # Update scan distance
        #             tracked_obstacles_copy[tracked_obstacles_key_copy[i]][2] = confirmed_scan_object[max_value_idx][2]
        #             # Append to deque list
        #             tracked_obstacles_copy[tracked_obstacles_key_copy[i]][3].append(
        #                 confirmed_scan_object[max_value_idx][1])
        #             # Get one time step
        #             tracked_obstacles_copy[tracked_obstacles_key_copy[i]][4] = time.time() - tracked_obstacles_copy[
        #                 tracked_obstacles_key_copy[i]][4]
        #             checked_obj_scans[max_value_idx] = True
        #         else:
        #             # Remove object from tracked list
        #             if len(self.tracked_obstacles) > i:
        #                 del self.tracked_obstacles[tracked_obstacles_key_copy[i]]
        #                 self.tracked_obstacles_keys.pop(i)

        #     else:
        #         continue

        # Add the "unadded" current detected object scans to the tracking list
        # if checked_obj_scans is not None and len(confirmed_scan_object) > 0:
        #     for i in range(len(checked_obj_scans)):
        #         if checked_obj_scans[i] == True:
        #             pass
        #         else:  # False or None
        #             if confirmed_scan_object[i][0] == 'o':
        #                 # print("NEW OBSTACLE")
        #                 unique_id = uuid4()
        #                 # Get tracked obstacles in the following format [<object type>, <coord list>, <deque coord
        #                 # list>]
        #                 _tracked_obs = confirmed_scan_object[i][:]
        #                 _tracked_obs.append(deque([confirmed_scan_object[i][1]]))
        #                 _tracked_obs.append(time.time())
        #                 _tracked_obs.append(-1)  # velocity
        #                 _tracked_obs.append([0.0, 0.0])  # velocity X and Y
        #                 # Assign a unique id to tracked obstacles
        #                 self.tracked_obstacles[unique_id] = _tracked_obs
        #             else:
        #                 pass
        #         self.tracked_obstacles_keys = list(self.tracked_obstacles.keys())
        #         tracked_obstacles_key_copy = self.tracked_obstacles_keys[:]

        # Obstacle velocity estimation
        # estimated_obstacle_vel = []
        # for i in range(len(tracked_obstacles_key_copy)):
        #     if tracked_obstacles_key_copy[i] in self.tracked_obstacles:
        #         if len(self.tracked_obstacles[tracked_obstacles_key_copy[i]][3]) > 1:
        #             _timelapse = self.tracked_obstacles[tracked_obstacles_key_copy[i]][4]
        #             _deque_pose_prev = self.tracked_obstacles[tracked_obstacles_key_copy[i]][3][0]
        #             _deque_pose_next = self.tracked_obstacles[tracked_obstacles_key_copy[i]][3][1]
        #             # Velocity computation
        #             _distance_change = math.hypot(_deque_pose_prev[1] - _deque_pose_next[1],
        #                                           _deque_pose_prev[0] - _deque_pose_next[0])
        #             # _timelapse = self.tracked_obstacles[tracked_obstacles_key_copy[i]][4]
        #             _velocity = _distance_change / _timelapse

        #             # Append velocity to dictionary (for CP-ttc)
        #             self.tracked_obstacles[tracked_obstacles_key_copy[i]][5] = _velocity

        # tracked_obstacles_copy = self.tracked_obstacles
        # tracked_obstacles_key_copy = self.tracked_obstacles_keys

        # Time to Collision computation with collision cone
        # collision_prob = []
        # ego_score_collision_prob = []
        # _ego_score_collision_prob = 0.0
        # if len(self.agent_pose_deque) == 2:
        #     agent_vel = utils.get_timestep_velocity(self.agent_pose_deque, self.agent_vel_timestep)

        #     # Get agent velocity in x and y from Twist message
        #     agent_vel_x = -1 * self.linear_twist.x * (math.cos(self.angular_twist.z))
        #     agent_vel_y = self.linear_twist.x * (math.sin(self.angular_twist.z))

        #     # agent_disp_x = agent_vel_x * self.agent_vel_timestep
        #     # agent_disp_y = agent_vel_y * self.agent_vel_timestep

        #     # agent_vel2 = self.linear_twist.x

        #     # self.agent_pose_deque[1][0] = self.agent_pose_deque[0][0] + agent_disp_x
        #     # self.agent_pose_deque[1][1] = self.agent_pose_deque[0][1] + agent_disp_y

        #     goal_vel = 0.0
        #     obstacle_vel = []

        #     if len(self.tracked_obstacles) == 0:
        #         obstacle_vel.append(0.0)
        #     else:
        #         for i in range(len(self.tracked_obstacles)):
        #             obstacle_vel.append(self.tracked_obstacles[self.tracked_obstacles_keys[i]][5])

        #     obstacle_vel = obstacle_vel[0]

        #     # Check if there is a change in pose detected
        #     agent_pose_change = self.agent_pose_deque.count(self.agent_pose_deque[0]) == len(self.agent_pose_deque)

        #     vo_agent_pose_x = self.agent_pose_deque[1][0]
        #     vo_agent_pose_y = self.agent_pose_deque[1][1]
        #     for i in range(len(tracked_obstacles_key_copy)):
        #         vo_change_x, vo_change_y = 0, 0
        #         # Compute change in x and y from obstacle velocity information (we use obs. pose deque)
        #         if len(self.tracked_obstacles[tracked_obstacles_key_copy[i]][3]) > 1:
        #             _last_pose = self.tracked_obstacles[tracked_obstacles_key_copy[i]][3][0]
        #             _curr_pose = self.tracked_obstacles[tracked_obstacles_key_copy[i]][3][1]
        #             vo_change_x = _last_pose[0] - _curr_pose[0]
        #             vo_change_y = _last_pose[1] - _curr_pose[1]
        #             obstacle_vel_x = vo_change_x / self.agent_vel_timestep
        #             obstacle_vel_y = vo_change_y / self.agent_vel_timestep
        #             self.tracked_obstacles[tracked_obstacles_key_copy[i]][6] = [obstacle_vel_x, obstacle_vel_y]

        #         # Compute new robot position based on resultant velocity (Vr - Vo). Use last deque pose because that
        #         # is where the robot is headed to. Then, check if this new position is within the Collision cone.
        #         vo_agent_pose_x = self.agent_pose_deque[1][0] + vo_change_x
        #         vo_agent_pose_y = self.agent_pose_deque[1][1] + vo_change_y

        #     distance_to_collision_point = None
        #     for i in range(len(tracked_obstacles_key_copy)):
        #         distance_to_collision_point = utils.get_collision_point(
        #             [self.agent_pose_deque[0], [vo_agent_pose_x, vo_agent_pose_y]],
        #             self.tracked_obstacles[
        #                 tracked_obstacles_key_copy[i]][1],
        #             0.178)  # Turtlebot3 Burger Robot width

        #         resultant_vel = agent_vel - obstacle_vel

        #         if distance_to_collision_point is not None:
        #             if resultant_vel == 0:
        #                 time_to_collision = 0
        #                 _collision_prob = 1.0 * (
        #                     utils.compute_general_collision_prob(  # original: 1.0 (before ablation)
        #                         self.tracked_obstacles[tracked_obstacles_key_copy[i]][2], self.max_scan_range,
        #                         self.min_scan_range))
        #             else:
        #                 time_to_collision = distance_to_collision_point / resultant_vel
        #                 # if time_to_collision != 0:  # added to remove floating by zero error during division
        #                 _ego_score_collision_prob = utils.compute_collision_prob(time_to_collision)
        #                 _collision_prob = 0.5 * (utils.compute_collision_prob(time_to_collision)) + 0.5 * (
        #                     # original: 0.5, 0.5 (before ablation)
        #                     utils.compute_general_collision_prob(
        #                         self.tracked_obstacles[tracked_obstacles_key_copy[i]][2], self.max_scan_range,
        #                         self.min_scan_range))
        #             collision_prob.append([_collision_prob] + self.tracked_obstacles[tracked_obstacles_key_copy[i]][1] \
        #                                   + self.tracked_obstacles[tracked_obstacles_key_copy[i]][6])
        #             ego_score_collision_prob.append(_ego_score_collision_prob)
        #             # Append collision probability to dictionary
        #             # self.tracked_obstacles[tracked_obstacles_key_copy[i]][6] = _collision_prob
        #         else:
        #             time_to_collision = None
        #             _ego_score_collision_prob = utils.compute_collision_prob(time_to_collision)
        #             _collision_prob = 0.5 * (utils.compute_collision_prob(time_to_collision)) + 0.5 * (
        #                 # original: 0.5, 0.5 (before ablation)
        #                 utils.compute_general_collision_prob(
        #                     self.tracked_obstacles[tracked_obstacles_key_copy[i]][2], self.max_scan_range,
        #                     self.min_scan_range))
        #             collision_prob.append([_collision_prob] + self.tracked_obstacles[tracked_obstacles_key_copy[i]][1] \
        #                                   + self.tracked_obstacles[tracked_obstacles_key_copy[i]][6])
        #             ego_score_collision_prob.append(_ego_score_collision_prob)
        #             # Append collision probability to dictionary
        #             # self.tracked_obstacles[tracked_obstacles_key_copy[i]][6] = _collision_prob

            # if len(collision_prob) == 0:
            #     self.collision_prob = 0.0
            #     self.ego_score_collision_prob = 0.0

            #     top_k_obstacles_pose_vel = []
            #     top_k_obstacle_collision_prob = []
            #     for i in range(self.k_obstacle_count):
            #         top_k_obstacles_pose_vel.append([self.position.x, self.position.y, 0, 0])

            #     # Flatten
            #     flattened_top_k_obstacles_pose_vel = []
            #     for i in range(len(top_k_obstacles_pose_vel)):
            #         flattened_top_k_obstacles_pose_vel += top_k_obstacles_pose_vel[i]

            #     self.closest_obstacle_pose_vel = flattened_top_k_obstacles_pose_vel

            # else:
            #     self.ego_score_collision_prob = max(ego_score_collision_prob)

            #     # Find top K obstacles with highest CP, sort according to first elem of sublist
            #     _top_k_collision_prob = sorted(collision_prob, key=lambda x: x[0], reverse=True)
            #     top_k_collision_prob = _top_k_collision_prob[-self.k_obstacle_count:]

            #     self.collision_prob = top_k_collision_prob[0][0]

            #     # Get pose (x, y) and vels (Vx, Vy) of top K obstacles
            #     top_k_obstacles_pose_vel = []
            #     top_k_obstacle_collision_prob = []
            #     for i in range(len(top_k_collision_prob)):
            #         top_k_obstacles_pose_vel.append(top_k_collision_prob[i][1:])  # remove CP
            #         top_k_obstacle_collision_prob.append(top_k_collision_prob[i][0])

            #     # Check if tracked obstacle is less than K, use placeholder values if it is
            #     if len(top_k_obstacles_pose_vel) < (self.k_obstacle_count * 4):
            #         diff = self.k_obstacle_count - len(top_k_obstacles_pose_vel)
            #         for i in range(diff):
            #             top_k_obstacles_pose_vel.append([self.position.x, self.position.y, 0, 0])

            #     # Flatten
            #     flattened_top_k_obstacles_pose_vel = []
            #     for i in range(len(top_k_obstacles_pose_vel)):
            #         flattened_top_k_obstacles_pose_vel += top_k_obstacles_pose_vel[i]

            #     self.closest_obstacle_pose_vel = flattened_top_k_obstacles_pose_vel
            #     # self.closest_obstacle_pose = _max_cp_closest_obs[1]
            #     # self.closest_obstacle_vel = _max_cp_closest_obs[6]

            # Visualize dynamic obstacles on RVIZ (For video demo/debugging) (for K = 8)
            # _obstacle_pose_text = [Marker(), Marker(), Marker(), Marker(), Marker(),
            #                        Marker(), Marker(), Marker(), Marker(), Marker()]
            # _obstacle_pose_shape = [Marker(), Marker(), Marker(), Marker(), Marker(),
            #                         Marker(), Marker(), Marker(), Marker(), Marker()]
            # _obstacle_pose_state = [False, False, False]
            # _robot_pose = [self.position.x, self.position.y, self.robot_yaw]
            # _yaw_change = self.robot_yaw - self.previous_yaw

            # # Obstacle RVIZ positions
            # for i in range(len(top_k_obstacles_pose_vel)):
            #     try:  # Handle index out of range error
            #         _obstacle_pose_text[i] = utils.create_rviz_visualization_text_marker(_obstacle_pose_text[i],
            #                                                                              _robot_pose,
            #                                                                              top_k_obstacles_pose_vel[i],
            #                                                                              top_k_obstacle_collision_prob[
            #                                                                                  i])
            #         _obstacle_pose_shape[i] = utils.create_rviz_visualization_shape_marker(_obstacle_pose_shape[i],
            #                                                                                _robot_pose,
            #                                                                                top_k_obstacles_pose_vel[i],
            #                                                                                top_k_obstacle_collision_prob[
            #                                                                                    i])
            #         if _obstacle_pose_text[i].pose.position.x != 0.0 and _obstacle_pose_text[i].pose.position.y != 0.0:
            #             _obstacle_pose_state[i] = True
            #     except IndexError:
            #         # print("Index out of range for rviz tracked obstacle publisher")
            #         pass
            #     continue

            # Goal position marker
            # _goal_pose_marker_obj = Marker()
            # _goal_pose_marker = utils.create_rviz_visualization_shape_marker(_goal_pose_marker_obj, _robot_pose,
            #                                                                  [0, 0], 0.0, mtype="robot")

            # # Waypoint position marker
            # _waypoint_pose_marker_obj = Marker()
            # _waypoint_pose_marker = utils.create_rviz_visualization_shape_marker(_waypoint_pose_marker_obj, _robot_pose,
            #                                                                      [0, 0], 0.0, mtype="robot",
            #                                                                      goal_pose=[
            #                                                                          self.waypoint_desired_point.x,
            #                                                                          self.waypoint_desired_point.y])

            # text_publishers = [self.pub_obs1_pose_text, self.pub_obs2_pose_text, self.pub_obs3_pose_text]
            # shape_publishers = [self.pub_obs1_pose_shape, self.pub_obs2_pose_shape, self.pub_obs3_pose_shape]
            # for i in range(len(_obstacle_pose_state)):
            #     text_publishers[i].publish(_obstacle_pose_text[i])
            #     shape_publishers[i].publish(_obstacle_pose_shape[i])
            #     if not _obstacle_pose_state[i]:
            #         text_publishers[i].publish(_goal_pose_marker)
            #         shape_publishers[i].publish(_goal_pose_marker)

            # Get DTGP and compute goal reaching collision probability (NOT USED)
            # distance_to_goal_point = utils.get_collision_point(self.agent_pose_deque, [self.original_desired_point.x,
            #                                                                            self.original_desired_point.y],
            #                                                    0.2)
            # resultant_goal_vel = agent_vel - goal_vel
            # if distance_to_goal_point is not None:
            #     if resultant_goal_vel == 0:
            #         time_to_goal = 0
            #         self.goal_reaching_prob = 0.0
            #     else:
            #         time_to_goal = distance_to_goal_point / resultant_goal_vel
            #         self.goal_reaching_prob = 1.0 * (utils.compute_collision_prob(time_to_goal)) + 0.0 * (
            #             utils.compute_general_collision_prob(self.get_distance_to_goal(self.position),
            #                                                  self.max_scan_range,
            #                                                  self.min_scan_range))
            # else:
            #     time_to_goal = None
            #     self.goal_reaching_prob = 1.0 * (utils.compute_collision_prob(time_to_goal)) + 0.0 * (
            #         utils.compute_general_collision_prob(self.get_distance_to_goal(self.position),
            #                                              self.max_scan_range,
            #                                              self.min_scan_range))

            # FIFO and reset time
            # if len(self.agent_pose_deque) > 1:
            #     self.agent_pose_deque.popleft()
            # # Reset timer
            # for i in range(len(tracked_obstacles_key_copy)):
            #     self.tracked_obstacles[tracked_obstacles_key_copy[i]][4] = time.time()
            # self.vel_t0 = -1

        # Get safety and ego safety violation counts
        # for i in range(len(obstacle_scans)):
        #     if obstacle_scans[i][2] < 0.140:  # 0.178 (robot dimension width) * 78.7% of width
        #         self.ego_safety_violation_count += 1
        #         break

        # if self.ego_score_collision_prob > 0.4:  # 0.4 <-- original, thesis
        #     self.social_safety_violation_count += 1

        # To compare against previous tracking list when the object recognition fails (lost tracking)
        # self.prev_tracked_obstacles = tracked_obstacles_copy
        # self.prev_tracked_obstacles_key = tracked_obstacles_key_copy

        if not self.done:
            if data_bumper:
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

        #Fungsi CNN
        cnn_result = utils.cnn(data_cam)

        state = (scan_range + goal_heading_distance + agent_position + agent_orientation + agent_velocity
                 + cnn_result + data_bumper)
        
        print("INI PANJANG STATE: ", len(state))

        # Round items in state to 2 decimal places
        state = list(np.around(np.array(state), 3))

        return state, self.done

    def compute_reward(self, state, step_counter, done):
        current_heading = state[359]
        current_distance = state[360]

        distance_difference = current_distance - self.previous_distance
        heading_difference = current_heading - self.previous_heading

        step_reward = -2  # step penalty
        htg_reward = 0
        dtg_reward = 0
        waypoint_reward = 0

        # Action reward
        if self.last_action == "FORWARD":
            self.forward_action_reward_count += 1
            action_reward = 5
        if self.last_action == "TURN_LEFT":
            self.left_turn_action_reward_count += 1
            action_reward = 1
        if self.last_action == "TURN_RIGHT":
            self.right_turn_action_reward_count += 1
            action_reward = 1
        if self.last_action == "STOP":
            self.stop_action_reward_count += 1
            action_reward = 1

        # Distance to goal reward
        if distance_difference > 0:
            self.dtg_penalty_count += 1
            dtg_reward = 0
        if distance_difference < 0:
            self.dtg_reward_count += 1
            dtg_reward = 1

        # Heading to goal reward
        if heading_difference > 0:
            if current_heading > 0 and self.previous_heading < 0:
                self.htg_reward_count += 1
                htg_reward = 1
            if current_heading < 0 and self.previous_heading < 0:
                self.htg_reward_count += 1
                htg_reward = 1
            if current_heading < 0 and self.previous_heading > 0:
                self.htg_reward_count += 1
                htg_reward = 1
            if current_heading > 0 and self.previous_heading > 0:
                self.htg_penalty_count += 1
                htg_reward = 0
        if heading_difference < 0:
            if current_heading < 0 and self.previous_heading > 0:
                self.htg_reward_count += 1
                htg_reward = 1
            if current_heading > 0 and self.previous_heading > 0:
                self.htg_reward_count += 1
                htg_reward = 1
            if current_heading > 0 and self.previous_heading < 0:
                self.htg_reward_count += 1
                htg_reward = 1
            if current_heading < 0 and self.previous_heading < 0:
                self.htg_penalty_count += 1
                htg_reward = 0

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
            waypoint_reward = 200
            print("Change desired point")
            print(self.waypoint_desired_point)

            # Check if waypoint is within the goal point
            if self.is_in_true_desired_position(self.waypoint_desired_point):
                self.waypoint_desired_point.x = self.original_desired_point.x
                self.waypoint_desired_point.y = self.original_desired_point.y
                print("Change desired point to actual goal point since it is near")
                print(self.waypoint_desired_point)

        non_terminating_reward = step_reward + dtg_reward + htg_reward + waypoint_reward  # + action_reward
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
                goal_reward = 200
                reward = goal_reward + non_terminating_reward
            else:
                rospy.loginfo("Collision!!")
                self.episode_failure = True
                self.episode_success = False
                collision_reward = -200
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

        data = None
        while data is None:
            try:
                data_laser = rospy.wait_for_message('scan', LaserScan, timeout=5)
                data_bumper = rospy.wait_for_message('bumper_contact', ContactsState, timeout=5)
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

        data_laser = None
        while data_laser is None:
            try:
                data_laser = rospy.wait_for_message('scan', LaserScan, timeout=5)
                data_bumper = rospy.wait_for_message('bumper_contact', ContactsState, timeout=5)
                data_cam = rospy.wait_for_message('camera/depth/image_raw', Image, timeout=5)
                bridge = CvBridge()
                data_cam = bridge.imgmsg_to_cv2(data_cam, desired_encoding='passthrough')
            except:
                pass

        # Get initial heading and distance to goal
        self.previous_distance = self.get_distance_to_goal(self.position)
        self.previous_heading = self.get_heading_to_goal(self.position, self.orientation)
        self.previous_yaw = 3.14
        state, _ = self.get_state(data_laser, data_bumper, data_cam) # <-------- BARU NYAMPE SINI

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