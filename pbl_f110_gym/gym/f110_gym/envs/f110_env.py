# MIT License

# Copyright (c) 2020 Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

'''
Author: Hongrui Zheng
'''

# gym imports
import math
import pathlib
import random
import logging

import gym
import yaml
from gym import error, spaces, utils
from gym.utils import seeding

# base classes
from f110_gym.envs.base_classes import Simulator
from f110_gym.envs.splinify import SplineTrack, Trajectory
from f110_gym.envs.rewards import *

# others
import numpy as np
import os
import time

# gl
import pyglet
pyglet.options['debug_gl'] = False
from pyglet import gl

# constants

# rendering
VIDEO_W = 600
VIDEO_H = 400
WINDOW_W = 1000
WINDOW_H = 800

# RL consts
PENALTY = -0.01

dir_path = str(pathlib.Path(__file__).parent.parent.parent.parent.resolve())

with open(dir_path + '/configs/car_params.yaml', 'r') as file:
    car_params = yaml.safe_load(file)


class F110Env(gym.Env, utils.EzPickle):
    """
    OpenAI gym environment for F1TENTH
    
    Env should be initialized by calling gym.make('f110_gym:f110-v0', **kwargs)

    Args:
        kwargs:
            startpose (float, default=np.array([[0, 0, 0]]))

            seed (int, default=12345): seed for random state and reproducibility
            
            map (str, default='vegas'): name of the map used for the environment. Currently, available environments include: 'berlin', 'vegas', 'skirk'. You could use a string of the absolute path to the yaml file of your custom map.
        
            map_ext (str, default='png'): image extension of the map image file. For example 'png', 'pgm'
        
            params (dict, default={'mu': 1.0489, 'C_Sf':, 'C_Sr':, 'lf': 0.15875, 'lr': 0.17145, 'h': 0.074, 'm': 3.74, 'I': 0.04712, 's_min': -0.4189, 's_max': 0.4189, 'sv_min': -3.2, 'sv_max': 3.2, 'v_switch':7.319, 'a_max': 9.51, 'v_min':-5.0, 'v_max': 20.0, 'width': 0.31, 'length': 0.58}): dictionary of vehicle parameters.
            mu: surface friction coefficient
            C_Sf: Cornering stiffness coefficient, front
            C_Sr: Cornering stiffness coefficient, rear
            lf: Distance from center of gravity to front axle
            lr: Distance from center of gravity to rear axle
            h: Height of center of gravity
            m: Total mass of the vehicle
            I: Moment of inertial of the entire vehicle about the z axis
            s_min: Minimum steering angle constraint
            s_max: Maximum steering angle constraint
            sv_min: Minimum steering velocity constraint
            sv_max: Maximum steering velocity constraint
            v_switch: Switching velocity (velocity at which the acceleration is no longer able to create wheel spin)
            a_max: Maximum longitudinal acceleration
            v_min: Minimum longitudinal velocity
            v_max: Maximum longitudinal velocity
            width: width of the vehicle in meters
            length: length of the vehicle in meters

            num_agents (int, default=2): number of agents in the environment

            timestep (float, default=0.01): physics timestep

            ego_idx (int, default=0): ego's index in list of agents
    """
    metadata = {'render.modes': ['human', 'human_fast', 'rgb_array']}

    def __init__(self, **kwargs):
        # kwargs extraction
        try:
            self.start_pose = np.array([[kwargs['sx'], kwargs['sy'], kwargs['stheta']]])
        except:
            self.start_pose = np.array([[0, 0, 0]])
        try:
            self.seed = kwargs['seed']
        except:
            self.seed = 12345
        try:
            self.map_name = kwargs['map']
            # different default maps
            if self.map_name == 'berlin':
                self.map_path = os.path.dirname(os.path.abspath(__file__)) + '/maps/berlin.yaml'
            elif self.map_name == 'skirk':
                self.map_path = os.path.dirname(os.path.abspath(__file__)) + '/maps/skirk.yaml'
            elif self.map_name == 'levine':
                self.map_path = os.path.dirname(os.path.abspath(__file__)) + '/maps/levine.yaml'
            else:
                self.map_path = self.map_name + '.yaml'
        except:
            self.map_path = os.path.dirname(os.path.abspath(__file__)) + '/maps/vegas.yaml'

        try:
            self.map_ext = kwargs['map_ext']
        except:
            self.map_ext = '.png'

        try:
            self.params = kwargs['params']
        except:
            self.params = {'mu': 1.0489, 'C_Sf': 4.718, 'C_Sr': 5.4562, 'lf': 0.15875, 'lr': 0.17145, 'h': 0.074, 'm': 3.74, 'I': 0.04712, 's_min': -0.4189, 's_max': 0.4189, 'sv_min': -3.2, 'sv_max': 3.2, 'v_switch': 7.319, 'a_max': 9.51, 'v_min':-5.0, 'v_max': 20.0, 'width': 0.31, 'length': 0.58}
        try:
            self.params_noise = kwargs['params_noise']
            if self.params_noise:
                try:
                    self.var_Csf = kwargs['var_Csf']
                    self.var_Csr = kwargs['var_Csr']
                    self.var_mu = kwargs['var_mu']
                except:
                    raise ValueError("When providing noise variances for the car, pleas provide all of them. One of them was not provided")
                try:
                    self.redraw_upon_reset = kwargs['redraw_upon_reset']
                except:
                    self.redraw_upon_reset = False
        except:
            self.params_noise = False
        
        # simulation parameters
        try:
            self.num_agents = kwargs['num_agents']
        except:
            self.num_agents = 2

        try:
            self.timestep = kwargs['timestep']
        except:
            self.timestep = 0.01

        # default ego index
        try:
            self.ego_idx = kwargs['ego_idx']
        except:
            self.ego_idx = 0

        # threshold for slow model use
        try:
            self.slow_threshold = kwargs['slow_threshold']
        except:
            self.slow_threshold = 0.5

        try:
            self._max_episode_length = kwargs['ep_len']
        except:
            self._max_episode_length = 10000

        self._cur_step = 0

        # radius to consider done
        self.start_thresh = 0.5  # 10cm

        # env states
        self.poses_x = []
        self.poses_y = []
        self.poses_theta = []
        self.collisions = np.zeros((self.num_agents, ))
        # TODO: collision_idx not used yet
        # self.collision_idx = -1 * np.ones((self.num_agents, ))

        # loop completion
        self.near_start = True
        self.num_toggles = 0

        # race info
        self.lap_times = np.zeros((self.num_agents, ))
        self.lap_counts = np.zeros((self.num_agents, ))
        self.current_time = 0.0

        # finish line info
        self.num_toggles = 0
        self.near_start = True
        self.near_starts = np.array([True]*self.num_agents)
        self.toggle_list = np.zeros((self.num_agents,))
        self.start_xs = np.zeros((self.num_agents, ))
        self.start_ys = np.zeros((self.num_agents, ))
        self.start_thetas = np.zeros((self.num_agents, ))
        self.start_rot = np.eye(2)

        # initiate stuff
        if not self.params_noise:
            self.sim = Simulator(self.params, self.num_agents, self.seed, slow_threshold=self.slow_threshold)
        else:
            par_variances = {'C_Sf': self.var_Csf, 'C_Sr': self.var_Csr, 'mu': self.var_mu}
            self.sim = Simulator(self.params, self.num_agents, self.seed, slow_threshold=self.slow_threshold, par_variances=par_variances, redraw_upon_reset=self.redraw_upon_reset)
        self.sim.set_map(self.map_path, self.map_ext)

        # rendering
        self.renderer = None
        self.current_obs = None

    def __del__(self):
        """
        Finalizer, does cleanup
        """
        pass

    def _check_done(self):
        """
        Check if the current rollout is done
        
        Args:
            None

        Returns:
            done (bool): whether the rollout is done
            toggle_list (list[int]): each agent's toggle list for crossing the finish zone
        """

        # this is assuming 2 agents
        # TODO: switch to maybe s-based
        left_t = 2
        right_t = 2
        
        poses_x = np.array(self.poses_x)-self.start_xs
        poses_y = np.array(self.poses_y)-self.start_ys
        delta_pt = np.dot(self.start_rot, np.stack((poses_x, poses_y), axis=0))
        temp_y = delta_pt[1,:]
        idx1 = temp_y > left_t
        idx2 = temp_y < -right_t
        temp_y[idx1] -= left_t
        temp_y[idx2] = -right_t - temp_y[idx2]
        temp_y[np.invert(np.logical_or(idx1, idx2))] = 0

        dist2 = delta_pt[0,:]**2 + temp_y**2
        closes = dist2 <= 0.1
        for i in range(self.num_agents):
            if closes[i] and not self.near_starts[i]:
                self.near_starts[i] = True
                self.toggle_list[i] += 1
            elif not closes[i] and self.near_starts[i]:
                self.near_starts[i] = False
                self.toggle_list[i] += 1
            self.lap_counts[i] = self.toggle_list[i] // 2
            if self.toggle_list[i] < 4:
                self.lap_times[i] = self.current_time
        
        done = (self.collisions[self.ego_idx]) or np.all(self.toggle_list >= 4) or self._cur_step >= self._max_episode_length
        #done = self.collisions[self.ego_idx] # modification only to make the env done when the car crashes, and not when 2 laps are done


        return bool(done), self.toggle_list >= 4

    def _update_state(self, obs_dict):
        """
        Update the env's states according to observations
        
        Args:
            obs_dict (dict): dictionary of observation

        Returns:
            None
        """
        self.poses_x = obs_dict['poses_x']
        self.poses_y = obs_dict['poses_y']
        self.poses_theta = obs_dict['poses_theta']
        self.collisions = obs_dict['collisions']

    def step(self, action):
        """
        Step function for the gym env

        Args:
            action (np.ndarray(num_agents, 2))

        Returns:
            obs (dict): observation of the current step
            reward (float, default=self.timestep): step reward, currently is physics timestep
            done (bool): if the simulation is done
            info (dict): auxillary information dictionary
        """

        self._cur_step += 1

        # call simulation step
        obs = self.sim.step(action)
        obs['lap_times'] = self.lap_times
        obs['lap_counts'] = self.lap_counts

        for key, item in obs.items():
            if key != 'ego_idx':
                obs[key] = np.array(item, dtype=np.float32)
        self.current_obs = obs


        # times
        reward = self.timestep
        self.current_time = self.current_time + self.timestep
        
        # update data member
        self._update_state(obs)

        # check done
        done, toggle_list = self._check_done()
        info = {'checkpoint_done': toggle_list}

        return obs, reward, done, info

    def reset(self, poses=-1):
        """
        Reset the gym environment by given poses

        Args:
            poses (np.ndarray (num_agents, 3)): poses to reset agents to

        Returns:
            obs (dict): observation of the current step
            reward (float, default=self.timestep): step reward, currently is physics timestep
            done (bool): if the simulation is done
            info (dict): auxillary information dictionary
        """
        self._cur_step = 0

        if type(poses) == int:
            if poses == -1:
                poses=self.start_pose

        # reset counters and data members
        self.current_time = 0.0
        self.collisions = np.zeros((self.num_agents, ))
        self.num_toggles = 0
        self.near_start = True
        self.near_starts = np.array([True]*self.num_agents)
        self.toggle_list = np.zeros((self.num_agents,))

        # states after reset
        self.start_xs = poses[:, 0]
        self.start_ys = poses[:, 1]
        self.start_thetas = poses[:, 2]
        self.start_rot = np.array([[np.cos(-self.start_thetas[self.ego_idx]), -np.sin(-self.start_thetas[self.ego_idx])], [np.sin(-self.start_thetas[self.ego_idx]), np.cos(-self.start_thetas[self.ego_idx])]])

        # call reset to simulator
        self.sim.reset(poses)
    

        # get no input observations
        action = np.zeros((self.num_agents, 2))
        obs, *_ = self.step(action)
        return obs

    def update_map(self, map_path, map_ext):
        """
        Updates the map used by simulation

        Args:
            map_path (str): absolute path to the map yaml file
            map_ext (str): extension of the map image file

        Returns:
            None
        """
        self.sim.set_map(map_path, map_ext)

    def update_params(self, params, index=-1):
        """
        Updates the parameters used by simulation for vehicles
        
        Args:
            params (dict): dictionary of parameters
            index (int, default=-1): if >= 0 then only update a specific agent's params

        Returns:
            None
        """
        self.sim.update_params(params, agent_idx=index)

    def render(self, mode='human'):
        """
        Renders the environment with pyglet. Use mouse scroll in the window to zoom in/out, use mouse click drag to pan. Shows the agents, the map, current fps (bottom left corner), and the race information near as text.

        Args:
            mode (str, default='human'): rendering mode, currently supports:
                'human': slowed down rendering such that the env is rendered in a way that sim time elapsed is close to real time elapsed
                'human_fast': render as fast as possible

        Returns:
            None
        """
        assert mode in ['human', 'human_fast', 'rgb_array']
        if self.renderer is None:
            # first call, initialize everything
            from f110_gym.envs.rendering import EnvRenderer
            self.renderer = EnvRenderer(WINDOW_W, WINDOW_H)
            self.renderer.update_map(self.map_name, self.map_ext)
        self.renderer.update_obs(self.current_obs)
        self.renderer.dispatch_events()
        self.renderer.on_draw()
        self.renderer.flip()
        if mode == 'human':
            time.sleep(0.005)
        elif mode == 'human_fast':
            pass
        
        # This part was stolen from the gym repo
        # to be precise, here: https://github.com/openai/gym/blob/master/gym/envs/classic_control/rendering.py
        if mode == 'rgb_array':
            buffer = pyglet.image.get_buffer_manager().get_color_buffer()
            image_data = buffer.get_image_data()
            arr = np.frombuffer(image_data.get_data(), dtype=np.uint8)
            # In https://github.com/openai/gym-http-api/issues/2, we
            # discovered that someone using Xmonad on Arch was having
            # a window of size 598 x 398, though a 600 x 400 window
            # was requested. (Guess Xmonad was preserving a pixel for
            # the boundary.) So we use the buffer height/width rather
            # than the requested one.
            arr = arr.reshape(buffer.height, buffer.width, 4)
            arr = arr[::-1, :, 0:3]
            if arr is None:
                print(arr)
        
            return arr
        return True

    def get_reward(self, rew_type: str = "vel"):
        if self.sim.agents[self.ego_idx].in_collision:
            return PENALTY
        else:
            if rew_type == 'vel':
                return get_reward_vel(self.sim, self.ego_idx)
            
            elif rew_type == 'time':
                reward, self.next_waypoint, self.last_visited_times = get_reward_time(self.sim, self.ego_idx, self.next_waypoint, self.last_visited_times, self.track, self.current_time, self.times)
                return reward
            
            elif rew_type == 'adv':
                return get_reward_adv(self.theta, self.prev_theta)
            
            elif rew_type == 'advel':
                return get_reward_adv(self.theta, self.prev_theta)*get_reward_vel(self.sim, self.ego_idx)
            
            elif rew_type == 'pseudocurr':
                vel_rew = get_reward_vel(self.sim, self.ego_idx)
                adv_rew = get_reward_adv(self.theta, self.prev_theta)
                time_rew, self.next_waypoint, self.last_visited_times = get_reward_time(self.sim, self.ego_idx, self.next_waypoint, self.last_visited_times, self.track, self.current_time, self.times)
                return get_pseudocurr_reward(time_rew, vel_rew, adv_rew, self.sim, self.ego_idx)

            elif rew_type == 'scaramuzza':
                safety_rew = get_reward_safety(self.sim, self.ego_idx, self.track, self.theta)
                adv_rew = get_reward_adv(self.theta, self.prev_theta)
                vel_rew = get_reward_vel(self.sim, self.ego_idx)
                return 100*adv_rew + vel_rew + safety_rew
            
            else:
                raise NotImplementedError("Reward type not implemented")

class RLF110Env(F110Env): 
    def __init__(self, random_init=False, visualize=False, **kwargs):
        super().__init__(**kwargs)

        map_points = self.map_path[:-len(".yaml")] + "_waypoints.txt"
        self.track = SplineTrack(map_points)

        trajectory_file = self.map_path[:-len(".yaml")] + "_trajectory_MPCC.txt"
        self.trajectory = Trajectory(trajectory_file)
        self.visualize = visualize

        try:
            self._observation_type = kwargs['obs_type']
        except:
            self._observation_type = 'original'
        
        self.prev_action = np.zeros((2,1))
        if self._observation_type == 'original':
            self.observation_space = spaces.Box(-np.inf*np.ones(79, dtype=np.float32), np.inf*np.ones(79, dtype=np.float32))
        elif self._observation_type == 'Frenet':
            self.observation_space = spaces.Box(-np.inf*np.ones(79, dtype=np.float32), np.inf*np.ones(79, dtype=np.float32))
        elif self._observation_type == 'Frenet_trajectory':
            # we have : the spatial trajectory (30 x 2) and the Frenet frame (6)
            self._traj_len = 30
            obs_dim = self._traj_len*2 + 5 + 73 + 2 # 73 is the scans, 2 is the previous input TODO improve these numbers i.e. no hard writing
            self.observation_space = spaces.Box(-np.ones(obs_dim, dtype=np.float32), np.ones(obs_dim, dtype=np.float32))
            

        self.action_space = spaces.Box(
            -np.ones(2),
            np.ones(2),
        )  # steer, speed


        rand_theta = 0
        self.start_param = 0
        self.random_init = random_init
        if random_init:
            rand_theta = random.randrange(0, int(self.track.track_length))
            #print("A new theta! -> {}".format(rand_theta))
            self.start_param = rand_theta
            self.start_pose = np.empty((0,3))
            for i in range(self.num_agents):
                # in f1 car are 8 meters from each other, if we consider their position projected on the mid line
                pos = np.concatenate((self.track.get_coordinate(rand_theta), self.track.get_angle(rand_theta)), axis=None)
                if i % 2:
                    pos[:2] += (np.array(self.track.get_coordinate(rand_theta, line='out')) - np.array(self.track.get_coordinate(rand_theta)))/3
                else:
                    pos[:2] += (np.array(self.track.get_coordinate(rand_theta, line='int')) - np.array(self.track.get_coordinate(rand_theta)))/3
                self.start_pose = np.append(self.start_pose, [pos], axis=0)
                rand_theta -= 0.8
            self.sim.reset(self.start_pose)

        position = self.sim.agents[self.ego_idx].state[:2]
        # this part of code works only for one car
        if self.num_agents == 1:
            self.theta = self.track.find_theta(position, rand_theta)
            self.next_waypoint = int(np.floor(self.theta)) + 1
            self.times = 40*np.ones((int(self.track.track_length)+1,))
            self.last_visited_times = -1*np.ones((int(self.track.track_length)+1,))

        self.obs_dict = self.create_obs_dict()
        self.last_obs_dict = {}
        self.is_reset = False

    def step(self, action):
        """
        Performs a step in the RL env by wrapping the official F110 env.
        To comply with OpenAI standards, the wrapped env step is altered in the observation and reward.
        """
        #Dirty hack for SB3 PPO
        self.is_reset = False

        if action.shape == (2,):
            action = action.reshape((1, 2))
        #print(action.shape)

        norm_action = action
        action = self.denorm_action(action)

        obs, _, done, info = super(RLF110Env, self).step(action=action)


        self.update_theta()

        reward = self.get_reward(rew_type='adv')

        if self._observation_type == 'Frenet':
            obs, reward = self.get_frenet_obs(obs, reward, action)
        elif self._observation_type == 'Frenet_trajectory':
            obs, reward = self.get_frenet_traj_obs(obs, reward, norm_action, action)
        else:
            if self._observation_type != "original":
                print("Observation type not recognised, original is being used")
            obs, reward = self.get_orig_obs(obs, action, reward)

        if self.visualize:
            self.save_obs_dict(obs)

        return obs, reward, done, info

    def reset(self, poses=-1):
        """
        Resets the RL Environment and performs an env step.
        Poses=-1 because the official F110 Env expects a pose, which is against OpenAI standards.
        It was modified to catch -1 and use the initial pose.
        """
        self._cur_step = 0

        if type(poses) == int:
            if poses == -1:
                poses = self.start_pose

        # reset counters and data members
        self.current_time = 0.0
        self.collisions = np.zeros((self.num_agents, ))
        self.num_toggles = 0
        self.near_start = True
        self.near_starts = np.array([True]*self.num_agents)
        self.toggle_list = np.zeros((self.num_agents,))

        # states after reset
        self.start_xs = poses[:, 0]
        self.start_ys = poses[:, 1]
        self.start_thetas = poses[:, 2]
        self.start_rot = np.array([[np.cos(-self.start_thetas[self.ego_idx]), -np.sin(-self.start_thetas[self.ego_idx])], [np.sin(-self.start_thetas[self.ego_idx]), np.cos(-self.start_thetas[self.ego_idx])]])


        # call reset to simulator
        if self.random_init:
            rand_theta = random.randrange(0, int(self.track.track_length))
            #print("A new theta! -> {}".format(rand_theta))
            self.start_param = rand_theta
            self.start_pose = np.empty((0,3)) 
            for i in range(self.num_agents):
                # in f1 car are 8 meters from each other, if we consider their position projected on the mid line
                pos = np.concatenate((self.track.get_coordinate(rand_theta), self.track.get_angle(rand_theta)), axis=None)
                if (i%2) == 1:
                    pos[:2] += ( np.array(self.track.get_coordinate(rand_theta, line='out'))- np.array(self.track.get_coordinate(rand_theta)))/3
                else:
                    delta = np.array(self.track.get_coordinate(rand_theta, line='int')) - np.array(self.track.get_coordinate(rand_theta))
                    pos[:2] += delta/3
                self.start_pose = np.append(self.start_pose, [pos], axis=0)
                rand_theta -= 0.8
        
        self.sim.reset(self.start_pose)
        position = self.sim.agents[self.ego_idx].state[:2]

        self.theta = self.track.find_theta(position, self.start_param)
        self.next_waypoint = int(np.floor(self.theta)) + 1
        if self.next_waypoint < 0:
            self.next_waypoint += int(self.track.track_length)
        self.last_visited_times = -1*np.ones((int(self.track.track_length)+1,))
        

        # get no input observations
        action = np.zeros((self.num_agents, 2))
        obs, *_ = self.step(action)

        if self.visualize:
            self.is_reset = True
            self.last_obs_dict = self.obs_dict.copy()
            self.obs_dict = self.create_obs_dict()

        return obs

    def render(self, mode='human', close=False):
        return super(RLF110Env, self).render(mode=mode)

    def denorm_action(self, action):
        """
        As the action is by default clipped within -1, 1, we need to rescale it to the proper size
        """

        v_min = 1
        v_max = 10

        if action.shape == (1, 2):
            action[0][0] = ((action[0][0] + 1)/2)*(self.params['s_max'] - self.params['s_min']) + self.params['s_min']
            action[0][1] = ((action[0][1] + 1)/2)*(v_max - v_min) + v_min
        else:
            action[0][0] = ((action[0][0] + 1)/2)*(self.params['s_max'] - self.params['s_min']) + self.params['s_min']
            action[1][0] = ((action[1][0] + 1)/2)*(v_max - v_min) + v_min

        return action

    def norm_observation(self, obs):
        """
        As the observation is clipped, we need to take the original one and clip it
        """

        for i in range(self._traj_len):
            obs[2*i] = ((obs[2*i] + self._traj_len)/(2*self._traj_len))*2 - 1 # rescaling of x postion in trajectory
            obs[2*i + 1] = ((obs[2*i + 1] + self._traj_len)/(2*self._traj_len))*2 - 1 # rescaling of y position in trajectory

        for i in range(73):
            idx = 2*self._traj_len + i
            obs[idx] = ((obs[idx])/(30))*2 - 1 # rescaling of scans
        
        

        # TODO numbers here are again kinda hardcoded
        obs[73 + 2*self._traj_len + 0] = ((obs[73 + 2*self._traj_len + 0] + 10*self.params['width'])/(2*10*self.params['width']))*2 - 1 # deviation from traj
        assert obs[73 + 2*self._traj_len + 0] <=1 and obs[73 + 2*self._traj_len + 0] >=-1, "Normalisation not good, {} is too much".format(obs[73 + 2*self._traj_len + 0])
        
        obs[73 + 2*self._traj_len + 1] = ((obs[73 + 2*self._traj_len + 1] + np.pi)/(2*np.pi))*2 - 1 # relative heading to trajectory
        assert obs[73 + 2*self._traj_len + 1] <=1 and obs[73 + 2*self._traj_len + 1] >=-1, "Normalisation not good, {} is too much".format(obs[73 + 2*self._traj_len + 1])
        
        obs[73 + 2*self._traj_len + 2] = ((obs[73 + 2*self._traj_len + 2] + 20)/(2*20))*2 - 1 # longitudinal velocity
        assert obs[73 + 2*self._traj_len + 2] <=1 and obs[73 + 2*self._traj_len + 2] >=-1, "Normalisation not good, {} is too much".format(obs[73 + 2*self._traj_len + 2])

        obs[73 + 2*self._traj_len + 3] = ((obs[73 + 2*self._traj_len + 3] + 20)/(2*20))*2 - 1 # lateral velocity
        assert obs[73 + 2*self._traj_len + 3] <=1 and obs[73 + 2*self._traj_len + 3] >=-1, "Normalisation not good, {} is too much".format(obs[73 + 2*self._traj_len + 3])

        # TODO we must do something for the shit bug that makes the car spin like crazy, as it also breaks this
        if obs[73 + 2*self._traj_len + 4] >1: 
            obs[73 + 2*self._traj_len + 4] = 1
        elif obs[73 + 2*self._traj_len + 4] < -1:
            obs[73 + 2*self._traj_len + 4] = -1
        obs[73 + 2*self._traj_len + 4] = ((obs[73 + 2*self._traj_len + 4] + 100)/(2*100))*2 - 1 # yaw rate
        assert obs[73 + 2*self._traj_len + 4] <=1 and obs[73 + 2*self._traj_len + 4] >=-1, "Normalisation not good, {} is too much".format(obs[73 + 2*self._traj_len + 4])

        # we are assuming actions are already normalised (right now they are)
        return obs

    def get_frenet_obs(self, obs, reward, action):
        obs = self.obs_preprocessing(obs)
        car_position = self.sim.agents[self.ego_idx].state[:2]
        car_angle = obs['poses_theta'][0]%(2*np.pi)

        # following line can be used if lidar but with less than 1080 scans is wanted
        filtered_scans = [obs['scans'][i*10] for i in range(108) if i >= 108/6 and i <= 108*5/6 ]
        
        new_obs = {'scans':np.array(filtered_scans)} #'scans': obs['scans']}
        #new_obs['param'] = [self.theta]
        new_obs['deviation'] = [np.linalg.norm(car_position-self.track.get_coordinate(self.theta))]
        rel_head = self.get_rel_head(obs)
        new_obs['rel_heading'] = [rel_head]
        new_obs['longitudinal_vel'] = [obs['linear_vels_x'][0]]
        new_obs['later_vel'] = [obs['linear_vels_y'][0]]
        new_obs['yaw_rate'] = [obs['ang_vels_z'][0]]

        max_dev = self.get_max_dev()
        if new_obs['deviation'][0] >= max_dev:
            reward = PENALTY

        keys = ['scans', 'deviation', 'rel_heading', 'longitudinal_vel', 'later_vel', 'yaw_rate']
        obs = np.concatenate([new_obs[key] for key in keys], axis=None).astype(np.float32)

        # penalty onsteep action deviation
        reward -=  0.001*np.linalg.norm(self.prev_action - action)
        #print("Delta: {}".format(np.linalg.norm(self.prev_action - action)))
        self.prev_action = action

        return obs, reward
            
    def get_frenet_traj_obs(self, obs, reward, norm_action, action):
        obs = self.obs_preprocessing(obs)
        car_position = self.sim.agents[self.ego_idx].state[:2]
        car_angle = obs['poses_theta'][0]%(2*np.pi)

        # following line can be used if lidar but with less than 1080 scans is wanted
        filtered_scans = [obs['scans'][i*10] for i in range(108) if i >= 108/6 and i <= 108*5/6 ]
        
        new_obs = {'scans':np.array(filtered_scans)} 
        #new_obs['param'] = [self.theta]
        th_on_traj = self.trajectory.find_theta(car_position, self.theta-100) # TODO align trajectory and track
        new_obs['deviation'] = [np.linalg.norm(car_position-self.trajectory.get_coordinate(th_on_traj))]
        rel_head = self.get_rel_head(obs)
        new_obs['rel_heading'] = [rel_head]
        new_obs['longitudinal_vel'] = [obs['linear_vels_x'][0]]
        new_obs['later_vel'] = [obs['linear_vels_y'][0]]
        new_obs['yaw_rate'] = [obs['ang_vels_z'][0]]

        central_line_deviation = np.linalg.norm(car_position-self.track.get_coordinate(self.theta))
        max_dev = self.get_max_dev()
        if central_line_deviation >= max_dev:
            reward = PENALTY

        keys = ['scans', 'deviation', 'rel_heading', 'longitudinal_vel', 'later_vel', 'yaw_rate']
        obs = np.concatenate([new_obs[key] for key in keys], axis=None).astype(np.float32)
        rot_matr = np.array([[np.cos(car_angle), np.sin(car_angle)],[-np.sin(car_angle), np.cos(car_angle)]]) # rotate the points so that x axis is aligned with car
        
        # TODO align trajectory with track so we do not need the -100 that is basically an impure trick
        trajectory = [rot_matr@(np.array(self.trajectory.get_coordinate(th-100))-car_position) for th in np.linspace(self.theta, self.theta+self._traj_len, num=self._traj_len)]
        obs = np.concatenate((trajectory, obs, norm_action), axis=None)
        obs = self.norm_observation(obs)

        # TODO make this printz cool (logging?) for testing reward shaping
        logging.info("Base reward: {}".format(reward))
        deviation_penalty = new_obs['deviation'][0]*0.001 
        logging.info("Penalty for deviation: {}. Actual deviation: {}".format(deviation_penalty, new_obs['deviation'][0]))
        reward -= deviation_penalty   # a penalty on the deviation from the "predicted horizon"

        # penalty on steep action deviation
        action_delta = np.linalg.norm(self.prev_action - action)
        action_penalty = 0.001*action_delta
        logging.info("Penalty for action delta: {}. Actual delta: {}".format(action_penalty, action_delta))
        reward -= action_penalty
        self.prev_action = action
        
        return obs, reward

    def get_orig_obs(self, obs, action, reward):
        """
        Takes a subset of the entire observation and flattens it for etter feeding into the RL network
        """
        obs = self.obs_preprocessing(obs)

        filtered_scans = [obs['scans'][i*10] for i in range(108) if i >= 108/6 and i <= 108*5/6 ]
        obs['scans'] = filtered_scans

        keys = ['scans', 'poses_x', 'poses_y', 'poses_theta', 'linear_vels_x', 'linear_vels_y', 'ang_vels_z']
        obs = np.concatenate([obs[key] for key in keys], axis=None).astype(np.float32)

        car_position = self.sim.agents[self.ego_idx].state[:2]
        deviation = [np.linalg.norm(car_position-self.track.get_coordinate(self.theta))]
        max_dev = self.get_max_dev()
        if deviation >= max_dev:
            reward = PENALTY

        # penalty onsteep action deviation
        reward -=  0.001*np.linalg.norm(self.prev_action - action)
        #print("Delta: {}".format(np.linalg.norm(self.prev_action - action)))
        self.prev_action = action
        
        return obs, reward

    def obs_preprocessing(self, obs):
        """
        Removes all the unnecessary parts from the observation
        """
        keys = ['scans', 'poses_x', 'poses_y', 'poses_theta', 'linear_vels_x', 'linear_vels_y', 'ang_vels_z']
        new_obs = {}
        for k in keys:
            new_obs[k] = obs[k]
        obs = new_obs
        obs['scans'] = obs['scans'][0]
        
        return obs
    
    def update_theta(self):
        """
        Updates the projection of the car on the track, using the previous car position as est estimate
        """
        car_position = self.sim.agents[self.ego_idx].state[:2]
        self.prev_theta = self.theta
        self.theta = self.track.find_theta(car_position, self.theta)

    def get_rel_head(self, obs):
        """
        Obtain heading of the car relative to the track
        """

        angle = self.track.get_angle(self.theta)
        if angle < 0:
            print("Really? angle should always be > 0")
            angle+=2*np.pi
        if not np.isnan(obs['poses_theta'][0]):
            car_angle = obs['poses_theta'][0]%(2*np.pi)
        else:
            car_angle = 0
        rel_head = car_angle - angle
        if rel_head>=np.pi:
            rel_head -= 2*np.pi
        elif rel_head<= -np.pi:
            rel_head += 2*np.pi

        return rel_head

    def get_max_dev(self):
        track_width = np.linalg.norm(np.array(self.track.get_coordinate(self.theta, line='int')) -  np.array(self.track.get_coordinate(self.theta, line='out')))
        return track_width/2 - 1.5*self.params['width']

    def create_obs_dict(self):
        obs_dict = {}
        obs_dict['trajectory']          =  []
        obs_dict['scan']                =  []
        obs_dict['param']               =  []
        obs_dict['deviation']           =  []
        obs_dict['rel_heading']         =  []
        obs_dict['longitudinal_vel']    =  []
        obs_dict['later_vel']           =  []
        obs_dict['yaw_rate']            =  []
        obs_dict['action1']             =  []
        obs_dict['action2']             =  []
        obs_dict['car_position']        =  []

        return obs_dict

    def save_obs_dict(self, obs):
        car_position = self.sim.agents[self.ego_idx].state[:2]

        self.obs_dict['trajectory'].append(obs[:60].copy().reshape(-1, 2) + car_position)
        self.obs_dict['scan'].append(obs[60:133])
        self.obs_dict['param'].append(obs[133])
        self.obs_dict['deviation'].append(obs[134])
        self.obs_dict['rel_heading'].append(obs[135])
        self.obs_dict['longitudinal_vel'].append(obs[136])
        self.obs_dict['later_vel'].append(obs[137])
        self.obs_dict['yaw_rate'].append(obs[138])
        self.obs_dict['action1'].append(obs[139])
        self.obs_dict['action2'].append(obs[140])
        self.obs_dict['car_position'].append(car_position)

class F110EnvDirect(F110Env):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.sim = Simulator(self.params, self.num_agents, self.seed, direct_input=True, slow_threshold=self.slow_threshold)
        self.sim.set_map(self.map_path, self.map_ext)
