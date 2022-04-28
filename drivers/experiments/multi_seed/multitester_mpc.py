import pathlib
import time

import gym
import yaml
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from MPC.drivers import MPCCDriver
from splinify.splinify import SplineTrack


cur_dir = str(pathlib.Path(__file__).parent.resolve())
maps_directory = '/maps/'
track_name = 'SOCHI' # only SOCHI, circle, squarish are available for the time being
dir_path = cur_dir + maps_directory + track_name

with open(dir_path + '.yaml', 'r') as file:
    track_info = yaml.safe_load(file)
    start_point = track_info['start_point']

with open(cur_dir+"/config/MPCC_debug.yaml", "r") as conf:
    DEBUG = yaml.safe_load(conf)

with open(cur_dir+"/config/MPC_params.yaml", "r") as conf:
    MPC_params = yaml.safe_load(conf)

with open(cur_dir+"/config/car_params.yaml", "r") as conf:
    car_params = yaml.safe_load(conf)

with open(cur_dir+"/config/mus.txt", "r") as file:
    mus = file.readline()
    mus = mus.split(", ")
    mus = [float(mu) for mu in mus]

with open(cur_dir+'/maps/SOCHI_trajectory_MPCC.txt', 'r') as file:
    # remove first line
    file.readline()

    coords = file.readlines()
    def parse(string):
        return [float(el) for el in string.split(",")]
    coords = [parse(el) for el in coords]
    coords = np.array(coords)

track = SplineTrack(cur_dir+'/maps/SOCHI_waypoints.txt')

SLOW_THRESHOLD = MPC_params['slow_threshold']

N_RUNS = 13
MAX_EP_LEN = 10000

x_pos_orig = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)
y_pos_orig = np.inf*np.ones((N_RUNS, MAX_EP_LEN), dtype=np.float32)
time_for_lap_orig = np.inf*np.ones((N_RUNS,1), dtype=np.float32)
crashed_orig = np.ones((N_RUNS,1), dtype=bool)

for i in range(N_RUNS):
    driver = MPCCDriver(dir_path + '_waypoints.txt', slow_threshold=SLOW_THRESHOLD)
    car_params['mu'] = mus[i]
    car_params['C_Sf'] = car_params['C_sf']
    car_params['C_Sr'] = car_params['C_sr']
    env = gym.make('f110_gym:f110direct-v0', map= dir_path, map_ext=".png", num_agents=1, slow_threshold=SLOW_THRESHOLD, params=car_params)
    obs = env.reset(poses=np.array([[0.08314340759386596, -0.0009153513356068288, 4.1446321]]))
    j = 0
    done = False
    info = None
    real_time = 0
    time1 = time.perf_counter()
    while not done:
        x_pos_orig[i,j] = env.sim.agents[0].state[0]
        y_pos_orig[i,j] = env.sim.agents[0].state[1]
        j += 1
        steer_v, acc, predictions, cur_speed, cur_angle, cur_steer, cur_theta, F_constr, f_constr, cur_max_P, soft_cons, info_solv = driver.process_observation_plot(obs)
        actions = np.array([[steer_v, acc]])
        obs, rew, done, info = env.step(actions)
        real_time += 0.01
        #env.render()
    crashed_orig[i] = not info['checkpoint_done']
    time_for_lap_orig[i] = real_time
    time2 = time.perf_counter()
    print('San Sim. elapsed time: {:.2f} Real elapsed time: {:.2f}'.format(real_time, time2-time1))

fig = plt.figure(figsize=(20,20))
ax = fig.add_subplot(1,1,1)

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
    
names = [ ["x_{}".format(i), "y_{}".format(i)] for i in range(N_RUNS) ]
names = np.concatenate(names, axis=None)

data_coords = np.concatenate((x_pos_orig, y_pos_orig), axis = 0)
data_coords = pd.DataFrame(data_coords.T, columns=names)
data_coords.to_csv('./coords_mpc.csv')

names_runs = ['times', 'crashes']
data_runs = np.concatenate((time_for_lap_orig, crashed_orig), axis=1)
data_runs = pd.DataFrame(data_runs, columns=names_runs)
data_runs.to_csv('./runs_data_mpc.csv')


plt.show()