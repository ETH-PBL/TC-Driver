import os
import gym
import yaml, time
import pathlib
from argparse import Namespace

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from stable_baselines3 import SAC
from stable_baselines3.common.env_util import make_vec_env

from splinify.splinify import SplineTrack


### PATHS
dir_path = pathlib.Path(__file__).parent.resolve()
configs_dir = os.path.join(dir_path, 'configs')
model_dir = os.path.join(dir_path, 'models')

#AVAILABLE modes = ['end2end', 'hierarchical'] maps = ['circle', 'SOCHI'] Architectures = ['PPO', 'SAC']
sb3_ppo_config = {'policy_type': 'MlpPolicy', 'total_timesteps': 1e6, 'gamma': 0.99}

with open(os.path.join(configs_dir, 'car_params.yaml'), 'r') as file:
    car_params = yaml.safe_load(file)
    car_params['C_Sf'] = car_params['C_sf']
    car_params['C_Sr'] = car_params['C_sr']

with open(os.path.join(configs_dir, 'SOCHI_trajectory_MPCC.txt'), 'r') as file:
    # remove first line
    file.readline()

    coords = file.readlines()
    def parse(string):
        return [float(el) for el in string.split(",")]
    coords = [parse(el) for el in coords]
    coords = np.array(coords)

track = SplineTrack(os.path.join(configs_dir, 'SOCHI_waypoints.txt'), safety_margin=1.5*0.31)
safety_track = SplineTrack(os.path.join(configs_dir, 'SOCHI_waypoints.txt'), safety_margin=2.5*0.31)

N_RUNS = 13
MAX_EP_LEN = 10000

def multi_test(exp_conf):
    env_conf = init_confs(exp_conf)

    # prepare the storage arrays
    x_pos_orig = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)
    y_pos_orig = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)
    vels_orig = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)
    steer_actions_orig = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)
    speed_actions_orig = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)
    time_for_lap_orig = np.inf*np.ones((N_RUNS,1), dtype=np.float32)
    crashed_orig = np.ones((N_RUNS,1), dtype=bool)
    
    x_pos_trtr = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)
    y_pos_trtr = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)
    vels_trtr = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)
    steer_actions_trtr = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)
    speed_actions_trtr = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)
    time_for_lap_trtr = np.inf*np.ones((N_RUNS,1), dtype=np.float32)
    crashed_trtr = np.ones((N_RUNS,1), dtype=bool)
    
    # extract the random frictions
    mus = np.zeros((N_RUNS))
    for i in range(len(mus)):
        scale = -1
        while scale<=-1 or scale >= 1:
            scale = np.random.normal()
        mus[i] = car_params['mu'] - 0.150 + scale*0.075
    print("params: {}".format(mus))
    with open('./mus.txt', 'w') as file:
        file.writelines(", ".join([str(mu) for mu in mus]) + "\n")

    for i in range(N_RUNS):
        # set the parameter
        car_params['mu'] = mus[i]
        env_conf['params'] = car_params
        env_conf['obs_type'] = 'original'


        
        env_orig = gym.make(id='f110_gym:f110rl-v0', **env_conf)
        env_orig.training = False
        done = False
        model_orig = SAC.load(os.path.join(model_dir, 'model_SOCHI_SAC_original_noise_True/best_model.zip'.format(env_conf['obs_type'], str(env_conf['params_noise']))), env=env_orig)

        laptime = 0.0
        start = time.perf_counter()
        obs = env_orig.reset()
        j = 0
        while not done:
            # saving trajectories
            car_state = env_orig.get_car_state()
            x_pos_orig[i,j] = car_state[0]
            y_pos_orig[i,j] = car_state[1]
            vels_orig[i,j] = car_state[3]

            # simulating
            action, _states = model_orig.predict(obs, deterministic=True)
            obs, step_reward, done, info = env_orig.step(action) 
            steer_actions_orig[i,j] = action[0]
            speed_actions_orig[i,j] = action[1]
            laptime += 0.01
            #env.render(mode='human_fast')
            j += 1
        time_for_lap_orig[i] = laptime
        crashed_orig[i] = not info['checkpoint_done']
        print('Sim elapsed time: {:.2f} Real elapsed time: {:.2f}'.format(laptime, time.perf_counter() - start))


        env_conf['obs_type'] = 'Frenet_trajectory'
        env_trtr = gym.make(id='f110_gym:f110rl-v0', **env_conf)
        env_trtr.training = False
        done = False
        model_trtr = SAC.load(os.path.join(model_dir, 'model_SOCHI_SAC_Frenet_trajectory_noise_True/best_model.zip'), env=env_trtr)

        laptime = 0.0
        start = time.perf_counter()
        obs = env_trtr.reset()
        j = 0
        while not done:
            # saving trajectories
            car_state = env_trtr.get_car_state()
            x_pos_trtr[i,j] = car_state[0]
            y_pos_trtr[i,j] = car_state[1]
            vels_trtr[i,j] = car_state[3]

            # simulating
            action, _states = model_trtr.predict(obs, deterministic=True)
            obs, step_reward, done, info = env_trtr.step(action) 
            steer_actions_trtr[i,j] = action[0]
            speed_actions_trtr[i,j] = action[1]
            laptime += 0.01
            #env.render(mode='human_fast')
            j += 1
        time_for_lap_trtr[i] = laptime
        crashed_trtr[i] = not info['checkpoint_done']
        print('Sim elapsed time: {:.2f} Real elapsed time: {:.2f}'.format(laptime, time.perf_counter() - start))

    fig = plt.figure(figsize=(20,20))
    ax = fig.add_subplot(1,2,1)

    runs_handles = [0 for _ in range(N_RUNS)]
    for i in range(N_RUNS):
        runs_handles[i], = ax.plot(x_pos_orig[i,:int(time_for_lap_orig[i]*100)], y_pos_orig[i, :int(time_for_lap_orig[i]*100)], color=(0.75,0,0.75,0.1)) # 'm' but transparent

    mpc_handle, = ax.plot(coords[:,0], coords[:,1], 'gray')
    n_points = 1000
    int_line = np.array([track.get_coordinate(th, line='int') for th in np.linspace(0,track.track_length, n_points)])
    out_line = np.array([track.get_coordinate(th, line='out') for th in np.linspace(0,track.track_length, n_points)])
    ax.plot(int_line[:, 0], int_line[:, 1], 'k')
    ax.plot(out_line[:, 0], out_line[:, 1], 'k')
    ax.legend([mpc_handle, runs_handles[0]], ['MPC w/o noise', 'end2end w/ noise'])

    ax = fig.add_subplot(1,2,2)
    runs_handles_tr = [0 for _ in range(N_RUNS)]
    for i in range(N_RUNS):
        runs_handles_tr[0], = ax.plot(x_pos_trtr[i,:int(time_for_lap_trtr[i]*100)], y_pos_trtr[i, :int(time_for_lap_trtr[i]*100)], color=(0.5,0,0.5,0.1)) # 'purple' but transparent

    mpc_handle_tr, = ax.plot(coords[:,0], coords[:,1], 'gray')
    n_points = 1000
    int_line = np.array([track.get_coordinate(th, line='int') for th in np.linspace(0,track.track_length, n_points)])
    out_line = np.array([track.get_coordinate(th, line='out') for th in np.linspace(0,track.track_length, n_points)])
    ax.plot(int_line[:, 0], int_line[:, 1], 'k')
    ax.plot(out_line[:, 0], out_line[:, 1], 'k')
    ax.legend([mpc_handle_tr, runs_handles_tr[0]], ['MPC w/o noise', 'trajectory tracking w/ noise'])
    
    # save sata with pandas 
    names = [ ["x_{}".format(i), "y_{}".format(i)] for i in range(N_RUNS) ]
    names = np.concatenate(names, axis=None)

    data_coords = np.concatenate((x_pos_orig, y_pos_orig), axis = 0)
    data_coords = pd.DataFrame(data_coords.T, columns=names)
    data_coords.to_csv('./coords_orig.csv')

    names_runs = ['times', 'crashes']
    data_runs = np.concatenate((time_for_lap_orig, crashed_orig), axis=1)
    data_runs = pd.DataFrame(data_runs, columns=names_runs)
    data_runs.to_csv('./runs_data_orig.csv')

    data_coords = np.concatenate((x_pos_trtr, y_pos_trtr), axis = 0)
    data_coords = pd.DataFrame(data_coords.T, columns=names)
    data_coords.to_csv('./coords_trtr.csv')

    data_runs = np.concatenate((time_for_lap_trtr, crashed_trtr), axis=1)
    data_runs = pd.DataFrame(data_runs, columns=names_runs)
    data_runs.to_csv('./runs_data_trtr.csv')
    
    plt.show()

def init_confs(exp_conf):
    
    with open(os.path.join(configs_dir, '{}.yaml'.format(exp_conf['map']))) as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
        conf = Namespace(**conf_dict)

    env_conf = {'map': os.path.join(configs_dir, '{}'.format(exp_conf['map'])),
                    'map_ext': conf.map_ext,
                    'random_init': False,
                    'sx': conf.sx,
                    'sy': conf.sy,
                    'stheta': conf.stheta,
                    'num_agents': 1,
                    'ep_len':MAX_EP_LEN, # this sets the maximum episode length, it is ~1.5 times the best time for a lap, so it changes from track to track
                    'obs_type':'original',
                    'params_noise':True,
                    'render_mode':'rgb_array'
                    }

    return env_conf

if __name__ == '__main__':
    exp_conf = {'mode': 'Frenet_trajectory', 'map': 'SOCHI', 'arch': 'SAC', 'conf': sb3_ppo_config}
    multi_test(exp_conf=exp_conf)
