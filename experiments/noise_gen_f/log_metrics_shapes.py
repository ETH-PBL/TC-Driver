import os
import pathlib

import gym
import time
import yaml
import numpy as np
import pandas as pd

from MPC.drivers import MPCCDriver
from splinify.splinify import SplineTrack
from index import Index


cur_dir = pathlib.Path(__file__).resolve().parent
root_dir = pathlib.Path(__file__).resolve().parents[2]
tracks_dir = os.path.join(root_dir, 'drivers/src/maps/')

with open('/home/invitedguest/edoardo-ghignone/drivers/src/config/MPC_params.yaml') as file:
    MPC_params = yaml.safe_load(file)

with open('/home/invitedguest/edoardo-ghignone/drivers/src/config/car_params.yaml') as file:
    car_params = yaml.safe_load(file)
    car_params['C_Sf'] = car_params['C_sf']
    car_params['C_Sr'] = car_params['C_sr'] 

with open('./thetas.txt') as file:
    line = file.readlines()
    thetas = [float(el.strip(', \n')) for el in line]
    N_RUNS = len(thetas)

with open('./mus.txt') as file:
    line = file.readline()
    mus = [float(mu) for mu in line.split(',')]
    assert N_RUNS == len(mus)

def main(max_time: int, track_shrink_coeff, track_name: str = 'SOCHI', mode: str = 'MPCC', start_par: int = 0):
    MAX_EP_LEN = max_time*100
    # prepare the storage arrays
    x_pos_mpc = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)
    y_pos_mpc = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)
    
    delta_mpc = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)
    vels_mpc = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)
    phi_mpc = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)
    phi_dot_mpc = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)
    beta_mpc = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)
    theta_mpc = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)

    acc_actions_mpc = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)
    stvel_actions_mpc = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)
    comp_time_mpc = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)
    oobounds_mpc = np.ones((N_RUNS,MAX_EP_LEN), dtype=bool)
    
    time_for_lap_mpc = np.inf*np.ones((N_RUNS,1), dtype=np.float32)
    crashed_mpc = np.ones((N_RUNS,1), dtype=bool)

    # init track, agent
    track_path = os.path.join(tracks_dir, track_name)
    track = SplineTrack(str(track_path) + '_waypoints.txt', safety_margin=track_shrink_coeff)

    env_car_params = car_params.copy()

    for exp_idx, (th, mu) in enumerate(zip(thetas, mus)):
        # initialise point
        start_point = track.get_coordinate(th)
        start_angle = track.get_angle(th)
        start_pose = np.concatenate((start_point, [start_angle]), axis = 0).reshape(1,-1)
        theta = th

        MPC_params['start_par'] = th
        driver = MPCCDriver(str(track_path) + '_waypoints.txt', track_shrink_coeff, MPC_params)
        
        env_car_params['mu'] = mu
        env = gym.make('f110_gym:f110direct-v0', map = track_path, map_ext=".png", num_agents=1)
        
        # reset track
        obs = env.reset(poses=start_pose)
        done = False
        info = {}
        info['checkpoint_done'] = 0
        i = 0
        while not done:
            state = env.sim.agents[0].state
            x_pos_mpc[exp_idx, i] = state[0]
            y_pos_mpc[exp_idx, i] = state[1]
            delta_mpc[exp_idx, i] = state[2]
            vels_mpc[exp_idx, i] = state[3]
            phi_mpc[exp_idx, i] = state[4]
            phi_dot_mpc[exp_idx, i] = state[5]
            beta_mpc[exp_idx, i] = state[6]

            theta = track.find_theta(state[:Index.S_Y+1], theta)
            theta_mpc[exp_idx, i] = theta

            flag_finished = info['checkpoint_done']
            flag_oo_soft = check_oo_soft(track, state, theta)
            oobounds_mpc[exp_idx, i] = flag_oo_soft
            theta += 0.01*state[Index.V]
            
            time1 = time.perf_counter()
            action = driver.process_observation(obs)
            time2 = time.perf_counter()
            acc_actions_mpc[exp_idx, i] = action[1]
            stvel_actions_mpc[exp_idx, i] = action[0]
            comp_time_mpc[exp_idx, i] = time2-time1

            obs, _, done, info = env.step(np.array([action]))
            # env.render()
            i += 1

        time_for_lap_mpc[exp_idx] = i/100
        crashed_mpc[exp_idx] = not info['checkpoint_done']

        state = env.sim.agents[0].state
        x_pos_mpc[exp_idx, i] = state[0]
        y_pos_mpc[exp_idx, i] = state[1]
        delta_mpc[exp_idx, i] = state[2]
        vels_mpc[exp_idx, i] = state[3]
        phi_mpc[exp_idx, i] = state[4]
        phi_dot_mpc[exp_idx, i] = state[5]
        beta_mpc[exp_idx, i] = state[6]

        theta = track.find_theta(state[:Index.S_Y+1], theta)
        flag_finished = info['checkpoint_done']
        flag_oo_soft = check_oo_soft(track, state, theta)
        
        theta_mpc[exp_idx, i] = theta
        oobounds_mpc[exp_idx, i] = flag_oo_soft
        comp_time_mpc[exp_idx, i] = 69

        # save sata with pandas 
        names = [ [
            "x_{}".format(i), 
            "y_{}".format(i),
            "delta_{}".format(i),
            "v_{}".format(i),
            "phi_{}".format(i),
            "phi_dot_{}".format(i),
            "beta_{}".format(i),
            "theta_{}".format(i),
            "stvel_{}".format(i),
            "acc_{}".format(i),
            "comp_{}".format(i),
            "oob_{}".format(i),
            ] for i in range(N_RUNS) ]
        names = np.concatenate(np.array(names).T, axis=None)
        data_coords = np.concatenate((
            x_pos_mpc, 
            y_pos_mpc, 
            delta_mpc, 
            vels_mpc, 
            phi_mpc, 
            phi_dot_mpc, 
            beta_mpc,
            theta_mpc, 
            stvel_actions_mpc,
            acc_actions_mpc,
            comp_time_mpc,
            oobounds_mpc
            ), axis = 0)
        data_coords = pd.DataFrame(data_coords.T, columns=names)
        data_coords.to_csv('./coords_mpc.csv')

        names_runs = ['times', 'crashes']
        data_runs = np.concatenate((time_for_lap_mpc, crashed_mpc), axis=1)
        data_runs = pd.DataFrame(data_runs, columns=names_runs)
        data_runs.to_csv('./runs_data_mpc.csv')


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
        'track_shrink_coeff' : 3,
        'track_name' : 'f',
        'mode' : 'MPCC',
        'start_par' : 7,
    }
    main(**params_dict)
    
