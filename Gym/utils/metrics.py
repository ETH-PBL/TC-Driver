import os
import time
import pathlib

import wandb
import numpy as np
import pandas as pd

from splinify.splinify import SplineTrack

from .index import Index


root_dir = pathlib.Path(__file__).parents[1].resolve()
tracks_dir = os.path.join(root_dir, 'configs/')

def evaluate_metrics(
    env, 
    agent, 
    max_tsteps, 
    track_shrink_coeff, 
    idx,
    track_name: str = 'SOCHI', 
    start_par: int = 0,
    ):
    print("Evaluating metrics")

    done = False
    track_path = os.path.join(tracks_dir, track_name)
    track = SplineTrack(str(track_path) + '_waypoints.txt', safety_margin=track_shrink_coeff)
    start_point = track.get_coordinate(start_par)
    start_angle = track.get_angle(start_par)
    start_pose = np.concatenate((start_point, [start_angle]), axis = 0).reshape(1,-1)
    theta = start_par
    obs = env.reset()

    states = np.empty((max_tsteps+1, 7), dtype=np.float32)
    actions = np.empty((max_tsteps+1, 2), dtype=np.float32)
    advancements = np.empty((max_tsteps+1, 1), dtype=np.float32)
    computation_times = np.empty((max_tsteps+1, 1), dtype=np.float32)
    flags = np.empty((max_tsteps+1, 2), dtype=np.int8) # lap_finished, out_of_soft_boundaries
    columns_names = [
        's_x', 's_y', 'delta', 'velocity', 'yaw', 'yaw_der', 'slip_angle',
        'steer_v', 'acceleration',
        'param', 
        'comp_time', 
        'lap_finished', 'out_of_soft',
        ]
    done = False
    info = [{}]
    info[0]['checkpoint_done'] = 0
    i = 0
    
    print("Gathering data")
    while not done:
        state = env.get_attr('sim')[0].agents[0].state
        states[i, :] = state

        theta = track.find_theta(state[:Index.S_Y+1], theta)
        flag_finished = info[0]['checkpoint_done']
        flag_oo_soft = check_oo_soft(track, state, theta)
        advancements[i] = theta
        flags[i, :] = [flag_finished, flag_oo_soft]
        theta += 0.01*state[Index.V]
        
        time1 = time.perf_counter()
        action, _ = agent.predict(obs)
        time2 = time.perf_counter()
        actions[i, :] = action
        computation_times[i] = time2-time1

        obs, _, done, info = env.step(np.array([action]))
        i += 1
    
    all_data = np.concatenate((states, actions, advancements, computation_times, flags), axis=1)
    all_data = all_data[:i, :]
    df = pd.DataFrame(all_data, columns=columns_names) # TODO save csv so wandb logs it 
    
    wandb_log = {}
    # wandb_log['full_df_{}'.format(idx)] = wandb.Table(dataframe=df)
    wandb_log['eval_advancement'] = df['param'].iloc[-1]
    wandb_log['soft_constraints_viol'] = df['out_of_soft'].sum()/(i+1)
    wandb_log['lap_time'] = (i+1)/100
    wandb_log['lap_finished'] = df['lap_finished'].iloc[-1]

    df['vx'] = df['velocity']*np.cos(df['slip_angle'] + df['yaw'])
    df['vy'] = df['velocity']*np.sin(df['slip_angle'] + df['yaw'])
    df['acc_rel_x'] = df['vx'].diff().fillna(0)/(0.01)
    df['acc_rel_y'] = df['vy'].diff().fillna(0)/(0.01)
    wandb_log['acc_on_car_{}'.format(idx)] = wandb.Table(dataframe=df[['acc_rel_y', 'acc_rel_x']]) # TODO fix formatting of table (prob no can do)

    return wandb_log


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
