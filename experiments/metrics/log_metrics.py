import os
import pathlib

import gym
import time
import yaml
import numpy as np
import pandas as pd

from MPC.drivers import MPCCDriver
from splinify.splinify import SplineTrack
from utils.index import Index


cur_dir = pathlib.Path(__file__).resolve().parent
root_dir = pathlib.Path(__file__).resolve().parents[2]
tracks_dir = os.path.join(root_dir, 'drivers/src/maps/')

def main(max_time: int, track_shrink_coeff, track_name: str = 'SOCHI', mode: str = 'MPCC', start_par: int = 0):
    track_path = os.path.join(tracks_dir, track_name)

    experiment_name = '{}_{}_1'.format(mode, track_name)

    track = SplineTrack(str(track_path) + '_waypoints.txt', safety_margin=track_shrink_coeff)
    start_point = track.get_coordinate(start_par)
    start_angle = track.get_angle(start_par)
    start_pose = np.concatenate((start_point, [start_angle]), axis = 0).reshape(1,-1)
    theta = start_par
    
    if mode == 'MPCC':
        driver = MPCCDriver(str(track_path) + '_waypoints.txt', track_shrink_coeff)
        env = gym.make('f110_gym:f110direct-v0', map = track_path, map_ext=".png", num_agents=1)

        obs = env.reset(poses=start_pose)

        # initialize buffers to save data
        states = np.empty((max_time*100, 7), dtype=np.float32)
        actions = np.empty((max_time*100, 2), dtype=np.float32)
        advancements = np.empty((max_time*100, 1), dtype=np.float32)
        computation_times = np.empty((max_time*100, 1), dtype=np.float32)
        flags = np.empty((max_time*100, 2), dtype=np.int8) # lap_finished, out_of_soft_boundaries
        columns_names = [
            's_x', 's_y', 'delta', 'velocity', 'yaw', 'yaw_der', 'slip_angle',
            'steer_v', 'acceleration',
            'param', 
            'comp_time', 
            'lap_finished', 'out_of_soft',
            ]

        done = False
        info = {}
        info['checkpoint_done'] = 0
        i = 0
        while not done:
            state = env.sim.agents[0].state
            states[i, :] = state

            theta = track.find_theta(state[:Index.S_Y+1], theta)
            flag_finished = info['checkpoint_done']
            flag_oo_soft = check_oo_soft(track, state, theta)
            advancements[i] = theta
            flags[i, :] = [flag_finished, flag_oo_soft]
            theta += 0.01*state[Index.V]
            
            time1 = time.perf_counter()
            action = driver.process_observation(obs)
            time2 = time.perf_counter()
            actions[i, :] = action
            computation_times[i] = time2-time1

            obs, _, done, info = env.step(np.array([action]))
            env.render()
            i += 1

        state = env.sim.agents[0].state
        states[i, :] = state
        theta = track.find_theta(state[:Index.S_Y+1], theta)
        flag_finished = info['checkpoint_done']
        flag_oo_soft = check_oo_soft(track, state, theta)
        advancements[i] = theta
        flags[i, :] = [flag_finished, flag_oo_soft]
        computation_times[i] = 69

    else: 
        raise NotImplementedError("Not yet implemented")

    all_data = np.concatenate((states, actions, advancements, computation_times, flags), axis=1)
    all_data = all_data[:i+1, :]
    df = pd.DataFrame(all_data, columns=columns_names)
    exp_dir = os.path.join(cur_dir, 'log/{}/'.format(experiment_name))
    # if folder does not exist, create it
    if not os.path.isdir(exp_dir):
        os.mkdir(exp_dir)

    save_path = os.path.join(exp_dir, 'data.txt')
    df.to_csv(save_path)
    run_data = {
        'run_name': experiment_name,
        'track_name': track_name,
        'controller_mode': mode,
        'safety_margin': track_shrink_coeff,
        'start_parameter': start_par
    }
    with open(os.path.join(exp_dir, 'run_data.txt'), 'w') as file:
        yaml.dump(run_data, file)


def check_oo_soft(track: SplineTrack, state, theta):
    """
    Checks if the car is out of the soft constraints
    """

    coord = np.array(state[:Index.S_Y+1])
    centr_coord = np.array(track.get_coordinate(theta, line='mid'))
    left_coord = np.array(track.get_coordinate(theta, line='int'))
    right_coord = np.array(track.get_coordinate(theta, line='out'))

    dist_left = np.linalg.norm(left_coord-coord)
    dist_right = np.linalg.norm(right_coord-coord)

    dist_car_c = np.linalg.norm(coord - centr_coord)
    if dist_left < dist_right:
        dist_bound_c = np.linalg.norm(left_coord - centr_coord)        
    else:
        dist_bound_c = np.linalg.norm(right_coord - centr_coord)

    return int(dist_car_c > dist_bound_c)

if __name__ == '__main__':
    # TODO implement cl args

    params_dict = {
        'max_time' : 50,
        'track_shrink_coeff' : 2.3,
        'track_name' : 'SOCHI',
        'mode' : 'MPCC',
        'start_par' : 7,
    }
    main(**params_dict)
    
