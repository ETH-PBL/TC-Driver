import pathlib
import time
from datetime import datetime

import gym
import yaml
import wandb
import numpy as np
import matplotlib.pyplot as plt
from gym.wrappers import Monitor
from matplotlib.animation import FuncAnimation

from splinify.splinify import SplineTrack
from src.MPC.drivers import MPCCDriver
#from src.test_drivers.drivers import Enemy, GapFollower
#from src.utils.utils import get_wandb_log


plot_MPC_horizon = True
plot_constr = True
plot_state_diff = False
plot_throttle = True
plot_speed = True
plot_angle = False 
plot_steer = True
plot_steer_sp = True
plot_thetas = True
plot_P_el = True


cur_dir = str(pathlib.Path(__file__).parent.resolve())
maps_directory = '/src/maps/'
track_name = 'SOCHI' # only SOCHI, circle, squarish, rbring, f, smile
dir_path = cur_dir + maps_directory + track_name

with open(dir_path + '.yaml', 'r') as file:
    track_info = yaml.safe_load(file)
    start_point = track_info['start_point']

with open(cur_dir+"/src/config/MPCC_debug.yaml", "r") as conf:
    DEBUG = yaml.safe_load(conf)

with open(cur_dir+"/src/config/MPC_params.yaml", "r") as conf:
    MPC_params = yaml.safe_load(conf)

with open(cur_dir+"/src/config/car_params.yaml", "r") as conf:
    car_params = yaml.safe_load(conf)

SLOW_THRESHOLD = MPC_params['slow_threshold']

track = SplineTrack(dir_path + '_waypoints.txt', safety_margin=0*0.31)
safety_track = SplineTrack(dir_path + '_waypoints.txt', safety_margin=1.5*0.31)

# driver = Enemy()
driver = MPCCDriver(dir_path + '_waypoints.txt', slow_threshold=SLOW_THRESHOLD)

use_MPCC = isinstance(driver, MPCCDriver)

if use_MPCC:
    N = driver.MPC_params['N']

start_point = np.concatenate((track.get_coordinate(MPC_params['start_par']), [track.get_angle(MPC_params['start_par'])]), axis=None)
poses = np.array([start_point])
#env = gym.make('f110_gym:f110rl-v0', map= dir_path, map_ext=".png", num_agents=1, sx= poses[0][0], sy=poses[0][1], stheta=poses[0][2])
car_params['C_Sf'] = car_params['C_sf'] # TODO fix this thing in yaml
car_params['C_Sr'] = car_params['C_sr'] 

car_params['mu'] -= 0 # limit is 0.075
env = gym.make('f110_gym:f110direct-v0', map= dir_path, map_ext=".png", num_agents=1, slow_threshold=SLOW_THRESHOLD, params=car_params)
env = Monitor(env,  directory=cur_dir+"/videos/", force=True)

obs = env.reset(poses=poses)

config_dict = {
    "slow_threshold": SLOW_THRESHOLD,
    "start_point": start_point, 
    "track_name": track_name,
    "driver": driver,
    "solver": DEBUG['solver'],
    "max_velocity": MPC_params['upper_bound']['v'],
    "debug_options": DEBUG,
    "mpc_params": MPC_params,
    "env_timestep": env.timestep
}
dt = datetime.now()
wandb.init(name="MPC_MPCC_{}_{}".format(track_name, dt.strftime("%Y_%m_%d_%H_%M_%S")), 
            project="MPC", 
            entity="forzapbl", 
            monitor_gym=True, 
            config=config_dict, 
            mode="disabled") # run or disabled


fig = plt.figure(figsize=(12,10))

if plot_MPC_horizon:
    n_points = 1000
    int_line = np.array([track.get_coordinate(th, line='int') for th in np.linspace(0,track.track_length, n_points)])
    out_line = np.array([track.get_coordinate(th, line='out') for th in np.linspace(0,track.track_length, n_points)])
    int_safety_line = np.array([safety_track.get_coordinate(th, line='int') for th in np.linspace(0,safety_track.track_length, n_points)])
    out_safety_line = np.array([safety_track.get_coordinate(th, line='out') for th in np.linspace(0,safety_track.track_length, n_points)])

    n_points_constr = 4
    constraints_x = np.zeros((n_points_constr, 2*N))
    constraints_y = np.zeros((n_points_constr, 2*N))

    # plot car
    ax_car = fig.add_subplot(1,2,1)
    ax_car.plot(int_line[:, 0], int_line[:, 1])
    ax_car.plot(out_line[:, 0], out_line[:, 1])
    ax_car.plot(int_safety_line[:, 0], int_safety_line[:, 1], ':')
    ax_car.plot(out_safety_line[:, 0], out_safety_line[:, 1], '--')
    car, = plt.plot([0], [0], 'o')
    trajectory, = plt.plot([-10,-10,-10,-10], [-25,-25,-25,-25], 'x')
    constraints = plt.plot(constraints_x, constraints_y)
    if plot_thetas:
        thetas = []
        theta_plt, = plt.plot(thetas, '*')

# plot error in one-timestep-forward prediction
if plot_state_diff:
    ax_2x = fig.add_subplot(6,2,2)
    ax_2y = fig.add_subplot(6,2,4)
    diff_x = []
    diff_y = []
    ax_2x.plot(diff_x)
    ax_2y.plot(diff_y)

# plot throttle (acceleration)
if plot_throttle:    
    ax_throttle = fig.add_subplot(6,2,8)
    throttle = []
    ax_throttle.plot(throttle)

    # plot speed on top of throttle
    if plot_speed:
        speed = []
        ax_throttle.plot(speed)

# plot angle (acceleration)
if plot_angle:
    ax_angle = fig.add_subplot(6,2,10)
    angles = []
    ax_angle.plot(angles)

# plot steer
if plot_steer:
    ax_steer = fig.add_subplot(6,2,6)
    steer = []
    ax_steer.plot(steer)

    if plot_steer_sp:
        steer_sp = []
        ax_steer.plot(steer_sp)

# plot max el of P matrix
if plot_P_el:
    ax_matr_val = fig.add_subplot(6,2,12)
    matr_vals = []
    ax_matr_val.plot(matr_vals)


indices = []
ind = 0
done = False
info = None

def update_plot(frame):
    # input()
    global obs, ind, trajectory, info 
    indices.append(ind)
    if use_MPCC:
        time1 = time.perf_counter()
        steer_v, acc, predictions, cur_speed, cur_angle, cur_steer, cur_theta, F_constr, f_constr, cur_max_P, soft_cons, info_solv = driver.process_observation_plot(obs)
        #wandb_log = get_wandb_log(steer_v, acc, predictions, cur_speed, cur_angle, cur_steer, cur_theta, F_constr, f_constr, cur_max_P, soft_cons, info_solv,  12)
        time2 = time.perf_counter()
        print("Tot processing time in s: {}".format(time2-time1))
        #wandb_log['tot_time'] = time2-time1
        #print(F_constr, f_constr)
        actions = np.array([[steer_v, acc]])
        #prev_pos = track.get_coordinate(cur_theta)
    else: 
        speed_in, steer_in =  driver.process_observation(obs['scans'], {'pose_x':obs['poses_x'][0], 'pose_y':obs['poses_y'][0], 'pose_theta':obs['poses_theta'][0], 'linear_vel_x':obs['linear_vels_x'][0]})
        
        wandb_log={}
        for k, v in obs.items():
            if k!='scans':
                wandb_log[k] = float(v[0])
        
        if info:
            for k, v in info['Frenet'].items():
                wandb_log[k] = float(v[0])

        print(wandb_log)
        

        actions = np.array([[steer_in, speed_in]])
        
    
    obs, rew, done, info = env.step(actions)
    
    # recording trajectory
    #wandb_log['position_x'] = obs['poses_x'][0]
    #wandb_log['position_y'] = obs['poses_y'][0]
    # env.render()

    if (frame%1) == 1:
        if plot_MPC_horizon:
            # update car position
            car.set_data([obs['poses_x'][0]], [obs['poses_y'][0]])
            # update prediction
            trajectory.set_data(predictions[0, 1:], predictions[1, 1:])
            if plot_constr:
                for i, el in enumerate(predictions[2, 1:]):
                    for j in range(n_points_constr):
                        angle = track.get_angle(el + 0.1*(j-(n_points_constr-1)/2), line = 'int' ) % np.pi
                        if angle >= np.pi/4 and angle < np.pi*3/4 :
                            constraints_y[j, 2*i] = track.get_coordinate(el + 0.1*(j-(n_points_constr-1)/2), line = 'int' )[1]
                            constraints_x[j, 2*i] = constraints_y[j, 2*i] * (- F_constr[2*i, 1]) / F_constr[2*i, 0] + f_constr[2*i]/ F_constr[2*i, 0]
                        else:
                            constraints_x[j, 2*i] = track.get_coordinate(el + 0.1*(j-(n_points_constr-1)/2), line = 'int' )[0]
                            constraints_y[j, 2*i] = constraints_x[j, 2*i] * (- F_constr[2*i, 0]) / F_constr[2*i, 1] + f_constr[2*i]/ F_constr[2*i, 1]

                        angle = track.get_angle(el + 0.1*(j-(n_points_constr-1)/2), line = 'out' ) % np.pi
                        if angle >= np.pi/4 and angle < np.pi*3/4 :
                            constraints_y[j, 2*i+1] = track.get_coordinate(el + 0.1*(j-(n_points_constr-1)/2), line = 'out' )[1]
                            constraints_x[j, 2*i+1] = constraints_y[j, 2*i+1] * (- F_constr[2*i+1, 1]) / F_constr[2*i+1, 0] + f_constr[2*i+1]/ F_constr[2*i+1, 0]
                        else:
                            constraints_x[j, 2*i+1] = track.get_coordinate(el + 0.1*(j-(n_points_constr-1)/2), line = 'out' )[0]
                            constraints_y[j, 2*i+1] = constraints_x[j, 2*i+1] * (- F_constr[2*i+1, 0]) / F_constr[2*i+1, 1] + f_constr[2*i+1]/ F_constr[2*i+1, 1]
                        
                    constraints[2*i].set_data(constraints_x[:, 2*i], constraints_y[:, 2*i])
                    constraints[2*i+1].set_data(constraints_x[:, 2*i+1], constraints_y[:, 2*i+1])
            
            if plot_thetas:
                thetas = np.array([track.get_coordinate(th) for th in predictions[2, :]])
                theta_plt.set_data(thetas[:,0], thetas[:, 1])
        
        # update error plot
        if plot_state_diff:
            diff_x.append(obs['poses_x'][0]-predictions[0, 0])
            diff_y.append(obs['poses_y'][0]-predictions[1, 0])
            ax_2x.clear()
            ax_2x.plot(diff_x)
            ax_2x.legend(['state_diff_x'])
            ax_2x.grid(axis='y')
            ax_2y.clear()
            ax_2y.plot(diff_y)
            ax_2y.legend(['state_diff_y'])
            ax_2y.grid(axis='y')

        
        if plot_throttle:
            throttle.append(acc)
            ax_throttle.clear()
            ax_throttle.plot(throttle)

            if plot_speed:
                speed.append(cur_speed)
                ax_throttle.plot(speed)

            ax_throttle.legend(['acc', 'speed'])
            ax_throttle.grid()

        if plot_angle:
            angles.append(cur_angle)
            ax_angle.clear()
            ax_angle.plot(angles)
            ax_angle.legend(['global angle'])

        if plot_steer:
            steer.append(cur_steer)
            ax_steer.clear()
            ax_steer.plot(steer)
            if plot_steer_sp:
                steer_sp.append(steer_v)
                ax_steer.plot(steer_sp)
            ax_steer.legend(['steer', 'steer_speed'])
            ax_steer.grid(axis='y')

        if plot_P_el:
            matr_vals.append(info_solv['solve_status'])
            ax_matr_val.clear()
            ax_matr_val.plot(matr_vals)
            ax_matr_val.legend(['max P element'])


    #wandb.log(wandb_log)

    #print("timestep: {}".format(ind))
    ind += 1
    #print("reward: {}".format(rew))


my_animatino = FuncAnimation(fig, update_plot, interval=0, repeat=True)

plt.show()
