# Mobile Robot Navigation Using Twin Delayed Deep Deterministic Policy Gradient (TD3)
The repository provides two main functions: training a TD3 model for mobile robot navigation in the Gazebo simulation and implementing the model in a real-world environment using Robotino 4. This work is the result of my Bachelor’s Thesis in Robotics and AI Engineering at Airlangga University.

## Contents

**Robotino Description**\
In this package, everything related to the robotino's internals is written and stored here. This includes the robot's mesh link, camera mesh, link textures, and the robot model which is written in a xacro file. In this xacro file, the robot is fully defined, including the position of each robot link, the joints of each link, and the features it can perform.

**Robotino Gazebo**\
The Gazebo environment structure encompasses the physical configuration of the arena or map. This folder contains information on the number of obstacle objects placed in the arena, the goal points of the mission, the placement of the arena boundaries, and the arena's lighting.

**Robotino RL Sim**\
This package contains the system dynamics configuration for the overall process of training and testing the model. These configurations include obtaining data from the simulation environment, executing the robot's movement based on the agent's output, calculating rewards, managing the overall simulation process, and processing training data to the learning machine to produce the most appropriate action output.

## Installation
1. **Preparation**
Open your ubuntu 20.04 terminal, type this to updating package index.\
`sudo apt-get update && apt-get upgrade` 
Install pip.\
`sudo apt-get install python3-pip`
Install required packages.\
`pip3 install torch tensorflow shapely`
2. **Install ROS1**
Follow instruction from [this](http://wiki.ros.org/noetic/Installation/Ubuntu ) official ROS page (*notes: please choose ros-noetic-desktop-full).\
If you need more explanation for ROS1 installation, you can watch [this](https://youtu.be/Qk4vLFhvfbI?si=n54lakIGoaFFJlqf) great video.
3. **Create and Setup a Catkin Workspace**
- Open terminal and create catkin folder.\
`mkdir catkin_ws`
- Go to catkin_ws folder and create new folder.\
`cd catkin_ws && mkdir src`
- Initiate catkin workspace.\
`catkin_make`
- Add this line code to automaticaly run if new shell is open.\
`echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`\
`source ~/.bashrc`
4. **Clone repository to your own local machine**
- Go to home folder.\
`cd`
- Clone this repository.\
`git clone https://github.com/anShape/Mobile-Robot-Navigation-Using-TD3.git`
- Go to cloned repo folder\
`cd Mobile-Robot-Navigation-Using-TD3`
- Move all the files to a packages folder.\
`mv CMakeLists.txt robotino_description/ two_robotino_rl_sim/ README.md robotino_gazebo/ utilities/ ~/catkin_ws/src/`


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

## Demo
**Simulation**\
![Stage 4](https://github.com/user-attachments/assets/63d4265d-5268-4998-9990-47481c753d28)

**Real implementation**\
![Stage 4 real](https://github.com/user-attachments/assets/47bcfc14-d9d9-407e-b03b-fe45fefe5482)


## Special Thanks
This work are inspired and modification from Zerosansan repo. Check his project [here](https://github.com/zerosansan/td3_ddpg_sac_dqn_qlearning_sarsa_mobile_robot_navigation).
