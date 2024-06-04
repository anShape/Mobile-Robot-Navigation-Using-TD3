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
# from environment_stage_1_nobonus import Env  # <-- used in latest work
from eeeks_environment_stage_2_nobonus import Env
# from environment_stage_1_original import Env  # For thesis

# Importing the library
import psutil
import timeit

if __name__ == '__main__':
    rospy.init_node('td3_training', anonymous=True)

    # Init environment
    max_step = rospy.get_param("/robotino/nsteps")
    env = Env(action_dim=2, max_step=max_step)
    stage_name = rospy.get_param("/robotino/stage_name")

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('two_robotino_rl_sim')
    result_outdir = pkg_path + '/src/results/td3' + '/' + stage_name
    model_outdir = pkg_path + '/src/models/td3' + '/' + stage_name
    actor_model_param_path = model_outdir + '/td3_actor_model_ep'
    critic1_model_param_path = model_outdir + '/td3_critic1_model_ep'
    critic2_model_param_path = model_outdir + '/td3_critic2_model_ep'

    # Remove log file if exist
    # utils.remove_logfile_if_exist(result_outdir, "td3_training")

    stage = 1
    resume_epoch = 0
    continue_execution = False
    learning = False
    # actor_resume_path = actor_model_param_path + str(resume_epoch)
    # critic1_resume_path = critic1_model_param_path + str(resume_epoch)
    # critic2_resume_path = critic2_model_param_path + str(resume_epoch)
    actor_resume_path = '/home/ihsan/catkin_ws/src/two_robotino_rl_sim/src/models/td3/training/stage_1_best_actor_ep1555_reward_404'
    critic1_resume_path = '/home/ihsan/catkin_ws/src/two_robotino_rl_sim/src/models/td3/training/stage_1_best_critic1_ep1555_reward_404'
    critic2_resume_path = '/home/ihsan/catkin_ws/src/two_robotino_rl_sim/src/models/td3/training/stage_1_best_critic2_ep1555_reward_404'
    actor_path = actor_resume_path + '.pt'
    critic1_path = critic1_resume_path + '.pt'
    critic2_path = critic2_resume_path + '.pt'
    k_obstacle_count = 3

    nepisodes = 5
    nsteps = 300
    actor_learning_rate = rospy.get_param("/robotino/actor_alpha")
    critic_learning_rate = rospy.get_param("/robotino/critic_alpha")
    discount_factor = rospy.get_param("/robotino/gamma")
    softupdate_coefficient = rospy.get_param("/robotino/tau")
    batch_size = 128
    memory_size = 1000000
    network_inputs =  579 # 560 cnn + 19 other v4
    hidden_layers = 256  # Hidden dimension v4
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
        
    best_reward = 0
    best_time = 120
    best_ego_safety = 100

    step_counter = 0
    time_lapse = 0
    for ep in range(resume_epoch, nepisodes):
        rospy.logwarn("EPISODE: " + str(ep + 1))
        cumulated_reward = 0
        social_safety_score = 0
        ego_safety_score = 0

        # Initialize the environment and get first state of the robot
        observation = env.reset()
        time.sleep(0.1)  # Give time for RL to reset the agent's position
        start_time = time.time()
        env.done = False
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
                state = next_state

            if done:
                time_lapse = time.time() - start_time
                ego_safety_score = env.get_ego_safety_violation_status(step + 1)

                if cumulated_reward > best_reward:
                    # save model weights and monitoring data every new best model found based on reward.
                    best_reward = cumulated_reward
                    
                if time_lapse < best_time:
                    # save model weights and monitoring data every new best model found based on time.
                    best_time = time_lapse

                if ego_safety_score < best_ego_safety:
                    # save model weights and monitoring data every new best model found based on ego score.
                    best_ego_safety = ego_safety_score
                    
                rospy.logwarn("DONE")
                
                data = [ep + 1, success_episode, cumulated_reward, step + 1, ego_safety_score, time_lapse]

                utils.record_data(data, result_outdir, "td3_test_trajectory")
                print("EPISODE REWARD: ", cumulated_reward)
                print("BEST REWARD: ", best_reward)
                print("EPISODE STEP: ", step + 1)
                print("EPISODE SUCCESS: ", success_episode)
                print("TIME LAPSE: ", time_lapse)
                print("EGO SAFETY SCORE: ", ego_safety_score)
                break
    env.reset()
