# Mobile Robot Navigation Using Twin Delayed Deep Deterministic Policy Gradient (TD3)
The repository provides two main functions: training a TD3 model for mobile robot navigation in the Gazebo simulation and implementing the model in a real-world environment using Robotino 4. This work is the result of my Bachelor’s Thesis in Robotics and AI Engineering at Airlangga University.

## Contents

### Robotino Description
In this package, everything related to the robotino's internals is written and stored here. This includes the robot's mesh link, camera mesh, link textures, and the robot model which is written in a xacro file. In this xacro file, the robot is fully defined, including the position of each robot link, the joints of each link, and the features it can perform.

### Robotino Gazebo
The Gazebo environment structure encompasses the physical configuration of the arena or map. This folder contains information on the number of obstacle objects placed in the arena, the goal points of the mission, the placement of the arena boundaries, and the arena's lighting.

### Robotino RL Sim
This package contains the system dynamics configuration for the overall process of training and testing the model. These configurations include obtaining data from the simulation environment, executing the robot's movement based on the agent's output, calculating rewards, managing the overall simulation process, and processing training data to the learning machine to produce the most appropriate action output.

### Utilities
You can call this package "additional." It contains files used to check various processes during development.

## Installation
Under development

## Usage
Type this code in your terminal:
1. Start ROSCORE 
`roscore` 

2. Launch Gazebo World
`roslaunch robotino_gazebo robotino_crowd_dense.launch`

3. Place your robot in the Gazebo World 
`roslaunch robotino_gazebo put_robotino_in_world_training.launch`

4. Start training with TD3 
`roslaunch two_robotino_rl_sim start_td3_training.launch`

## Contribute
Under development