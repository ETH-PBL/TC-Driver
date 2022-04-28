import time
import pathlib

import gym
import numpy as np
from threading import Thread
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from pkg.src.pkg.drivers import MPCCDriver
from pkg.src.pkg.utils.splinify import SplineTrack

plot_state_diff = False
plot_throttle = True

cur_dir = str(pathlib.Path(__file__).parent.resolve())
track = SplineTrack(cur_dir + '/pkg/src/pkg/maps/SOCHI_waypoints.txt', safety_margin=1.5*0.31)
safety_track = SplineTrack(cur_dir + '/pkg/src/pkg/maps/SOCHI_waypoints.txt', safety_margin=2.5*0.31)

driver = MPCCDriver()
N = driver.MPC_params['N']
poses = np.array([[0.8162458, 1.1614572, 4.1446321]])
env = gym.make('f110_gym:f110-v0', map= cur_dir + '/pkg/src/pkg/maps/SOCHI', map_ext=".png", num_agents=1)

obs, _, done, info = env.reset(poses=poses)

n_points = 1000
int_line = np.array([track.get_coordinate(th, line='int') for th in np.linspace(0,track.track_length, n_points)])
out_line = np.array([track.get_coordinate(th, line='out') for th in np.linspace(0,track.track_length, n_points)])
int_safety_line = np.array([safety_track.get_coordinate(th, line='int') for th in np.linspace(0,safety_track.track_length, n_points)])
out_safety_line = np.array([safety_track.get_coordinate(th, line='out') for th in np.linspace(0,safety_track.track_length, n_points)])

n_points_constr = 4
constraints_x = np.zeros((n_points_constr, 2*N))
constraints_y = np.zeros((n_points_constr, 2*N))

# plot car
fig = plt.figure(1, figsize=(15, 15))
plt.plot(int_line[:, 0], int_line[:, 1])
plt.plot(out_line[:, 0], out_line[:, 1])
plt.plot(int_safety_line[:, 0], int_safety_line[:, 1], ':')
plt.plot(out_safety_line[:, 0], out_safety_line[:, 1], '--')
car, = plt.plot([0], [0], 'o')
trajectory, = plt.plot([1,1,1,1], [1,2,3,4], 'x')
constraints = plt.plot(constraints_x, constraints_y)

# plot error in one-timestep-forward prediction
if plot_state_diff:
    fig_2 = plt.figure(2, figsize=(10, 10))
    ax_2x = fig_2.add_subplot(2,1,1)
    ax_2y = fig_2.add_subplot(2,1,2)
    diff_x = []
    diff_y = []
    ax_2x.plot(diff_x)
    ax_2y.plot(diff_y)
    
# plot throttle (acceleration)
if plot_throttle:    
    fig_3 = plt.figure(2, figsize=(10, 5))
    ax_throttle = fig_3.add_subplot(1,1,1)
    throttle = []
    ax_throttle.plot(throttle)

indices = []

ind = 0
def update_plot(frame):
    global obs, ind, trajectory 

    steer, speed, predictions, F_constr, f_constr = driver.process_observation_plot(obs)
    actions = np.array([[steer, speed]])
    obs, _, done, info = env.step(actions)
    env.render()
    # update car position
    
    car.set_data(obs['poses_x'][0], obs['poses_y'][0])
    # update prediction
    trajectory.set_data(predictions[0, 1:], predictions[1, 1:])
    for i, el in enumerate(predictions[0, 1:]):
    	for j in range(n_points_constr):
    		constraints_x[j, 2*i] = el + 0.5*(j-(n_points_constr-1)/2)
    		constraints_x[j, 2*i+1] = constraints_x[j, 2*i]
    		constraints_y[j, 2*i] = constraints_x[j, 2*i] * (- F_constr[2*i, 0]) / F_constr[2*i, 1] + f_constr[2*i]/ F_constr[2*i, 1]
    		constraints_y[j, 2*i+1] = constraints_x[j, 2*i+1] * (- F_constr[2*i+1, 0]) / F_constr[2*i+1, 1] + f_constr[2*i+1]/ F_constr[2*i+1, 1]
    	constraints[2*i].set_data(constraints_x[:, 2*i], constraints_y[:, 2*i])
    	constraints[2*i+1].set_data(constraints_x[:, 2*i+1], constraints_y[:, 2*i+1])
    # update error plot
    if plot_state_diff:
        diff_x.append(obs['poses_x'][0]-predictions[0, 0])
        diff_y.append(obs['poses_y'][0]-predictions[1, 0])
        ax_2x.clear()
        ax_2x.plot(diff_x)
        ax_2y.clear()
        ax_2y.plot(diff_y)
    
    if plot_throttle:
        throttle.append(speed)
        ax_throttle.clear()
        ax_throttle.plot(throttle)
    
    print("timestep: {}".format(ind))
    ind += 1
    return trajectory

my_animatino = FuncAnimation(fig, update_plot, interval=0, repeat=True)

plt.show()

