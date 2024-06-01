#!/usr/bin/env python3

"""
Based on:
https://github.com/dranaju/project
"""
import td3
import rospy
import numpy as np
import rospkg
import utils
import time
import requests

from environment_real import Env

import sys
import signal

def main():

    # Init environment
    max_step = rospy.get_param("/robotino/nsteps")
    env = Env(action_dim=2, max_step=max_step)
    stage_name = rospy.get_param("/robotino/stage_name")

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('robotino_rl_sim')
    result_outdir = pkg_path + '/src/results/td3' + '/' + stage_name
    model_outdir = pkg_path + '/src/models/td3' + '/' + stage_name

    # Remove log file if exist
    # utils.remove_logfile_if_exist(result_outdir, "td3_training")

    actor_path = "/home/ihsan/catkin_ws/src/two_robotino_rl_sim/src/models/td3/training/stage_4_best_actor_ep483_reward_187.pt"
    critic1_path = "/home/ihsan/catkin_ws/src/two_robotino_rl_sim/src/models/td3/training/stage_4_best_critic1_ep483_reward_187.pt"
    critic2_path = "/home/ihsan/catkin_ws/src/two_robotino_rl_sim/src/models/td3/training/stage_4_best_critic2_ep483_reward_187.pt"

    ep = 0

    nsteps = rospy.get_param("/robotino/nsteps")
    actor_learning_rate = rospy.get_param("/robotino/actor_alpha")
    critic_learning_rate = rospy.get_param("/robotino/critic_alpha")
    discount_factor = rospy.get_param("/robotino/gamma")
    softupdate_coefficient = rospy.get_param("/robotino/tau")
    batch_size = 128
    memory_size = 1000000
    network_inputs = 579
    hidden_layers = 256  # Hidden dimension
    network_outputs = 2  # Action dimension
    action_v_max = 0.22  # m/s
    action_w_max = 2.0  # rad/s
    # resume_epoch = 100
    noise_std = 0.2
    noise_clip = 0.5
    policy_update = 2

    td3_trainer = td3.Agent(network_inputs, network_outputs, hidden_layers, actor_learning_rate,
                            critic_learning_rate, batch_size, memory_size, discount_factor,
                            softupdate_coefficient, action_v_max, action_w_max, noise_std, noise_clip,
                            policy_update)
    td3_trainer.load_models(actor_path, critic1_path, critic2_path)

    step_counter = 0
    time_lapse = 0

    cumulated_reward = 0
    social_safety_score = 0
    ego_safety_score = 0
    env.done = False

    rospy.logwarn("EPISODE: " + str(ep + 1))

    # Initialize the environment and get first state of the robot
    observation = env.reset()
    time.sleep(0.1)  # Give time for RL to reset the agent's position
    start_time = time.time()
    state = observation

    for step in range(nsteps):
        rospy.logwarn("EPISODE: " + str(ep + 1) + " | STEP: " + str(step + 1))
        step_counter += 1
        state = np.float32(state)
        action = td3_trainer.act(state, step, add_noise=True)
        _action = action.flatten().tolist()
        observation, reward, done = env.step(_action, step + 1, mode="continuous")
        success_episode, failure_episode = env.get_episode_status()
        cumulated_reward += reward

        next_state = observation
        next_state = np.float32(next_state)

        if not done:
            # rospy.logwarn("NOT DONE")
            state = next_state

        if done:
            time_lapse = time.time() - start_time
            ego_safety_score = env.get_ego_safety_violation_status(step + 1)
            # Debugging purposes
            if (step + 1) <= 2:
                env.shutdown()
                # raw_input("Press Enter to continue...")
            if (ep + 1) % 100 == 0:
                # save model weights and monitoring data every 100 epochs.
                td3_trainer.save_actor_model(model_outdir, "td3_actor_model_ep" + str(ep + 1) + '.pt')
                td3_trainer.save_critic1_model(model_outdir, "td3_critic1_model_ep" + str(ep + 1) + '.pt')
                td3_trainer.save_critic2_model(model_outdir, "td3_critic2_model_ep" + str(ep + 1) + '.pt')
            rospy.logwarn("DONE")
    
            data = [ep + 1, success_episode, failure_episode, cumulated_reward, step + 1, ego_safety_score,
                        social_safety_score, time_lapse]
            utils.record_data(data, result_outdir, "td3_training_trajectory_test")
            print("EPISODE REWARD: ", cumulated_reward)
            print("EPISODE STEP: ", step + 1)
            print("EPISODE SUCCESS: ", success_episode)
            print("EPISODE FAILURE: ", failure_episode)

            break
    
    requests.post('http://192.168.0.101/data/omnidrive', json=[0,0,0])  


def signal_handler(sig, frame):
    print('Menghentikan program...')
    # Lakukan pembersihan yang diperlukan di sini
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
