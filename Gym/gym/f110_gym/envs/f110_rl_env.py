import random
import logging

import numpy as np
from gym import spaces
from splinify.splinify import SplineTrack

from f110_gym.envs import normalisation
from f110_gym.envs.f110_env import F110Env
from f110_gym.envs.base_classes import Simulator


PENALTY = -1


class RLF110Env(F110Env): 
    def __init__(self, random_init=False, visualize=False, use_lidar=False, **kwargs):
        super().__init__(**kwargs)

        map_points = self.map_path[:-len(".yaml")] + "_waypoints.txt"
        try:
            self._use_traj = kwargs['use_trajectory']
        except:
            self._use_traj = False
        
        # track initialisation
        if self._use_traj:
            try:
                trajectory_file = self.map_path[:-len(".yaml")] + "_trajectory_MPCC.txt"
                self.track = SplineTrack(
                        path_to_file=map_points,
                        path_to_traj=trajectory_file
                    )
                print("Using trajectory")
            except:
                self.track = SplineTrack(map_points)
                print("Using center line")
        else:
            self.track = SplineTrack(map_points)
            print("Using center line")

        self.visualize = visualize

        try:
            self._observation_type = kwargs['obs_type']
        except:
            self._observation_type = 'original'

        try: 
            self._traj_coeff = kwargs['traj_coeff']
        except:
            self._traj_coeff = 0.05

        try:
            self._act_pen = kwargs['action_pen']
        except:
            self._act_pen = 0.001

        try: 
            self._angle_limit = kwargs['angle_limit']
        except:
            self._angle_limit = np.pi

        self.use_lidar = use_lidar
        
        self.prev_action = np.zeros((2,1))
        if self._observation_type == 'original':
            self.observation_space = spaces.Box(-np.inf*np.ones(16, dtype=np.float32), np.inf*np.ones(16, dtype=np.float32))
        elif self._observation_type == 'Frenet':
            # 18 comes from 11 lidar + 5 frenet
            self.observation_space = spaces.Box(-np.ones(16, dtype=np.float32), np.ones(16, dtype=np.float32))
        elif self._observation_type == 'Frenet_trajectory':
            # we have : the spatial trajectory (15 x 2) and the Frenet frame (2)
            self._traj_len = 15
            if self.use_lidar:
                obs_dim = self._traj_len*2 + 5 + 11 + 2 # 11 is the scans, 2 is the previous input TODO improve these numbers i.e. no hard writing
            else:
                obs_dim = self._traj_len*2 + 5 + 2 # no lidar version
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

        norm_action = action.copy()
        action = self.denorm_action(action)

        obs, _, done, info = super(RLF110Env, self).step(action=action)


        self.update_theta()

        reward = self.get_reward(rew_type='adv')

        if self._observation_type == 'Frenet':
            obs, reward = self.get_frenet_obs(obs, reward, action)
            info_obs = {'bad_obs': False}
        elif self._observation_type == 'Frenet_trajectory':
            obs, reward, info_obs = self.get_frenet_traj_obs(obs, reward, norm_action, action)
            if np.isnan(obs).any():
                raise ValueError("Nans observed in the observation!")
        else:
            if self._observation_type != "original":
                print("Observation type not recognised, original is being used")
            obs, reward = self.get_orig_obs(obs, action, reward)
            info_obs = {'bad_obs': False}

        if self.visualize:
            self.save_obs_dict(obs)

        if info_obs['bad_obs']:
            # interrupt sim if the observation is not valid
            logging.warning("Incorrect observation, terminating simulation")
            done = True

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
                """
                this version implements a random start but with f1 grid positioning, for random training it is not what we wanted

                # in f1 car are 8 meters from each other, if we consider their position projected on the mid line

                pos = np.concatenate((self.track.get_coordinate(rand_theta), self.track.get_angle(rand_theta)), axis=None)
                if (i%2) == 1:
                    pos[:2] += ( np.array(self.track.get_coordinate(rand_theta, line='out'))- np.array(self.track.get_coordinate(rand_theta)))/3
                else:
                    delta = np.array(self.track.get_coordinate(rand_theta, line='int')) - np.array(self.track.get_coordinate(rand_theta))
                    pos[:2] += delta/3
                self.start_pose = np.append(self.start_pose, [pos], axis=0)
                rand_theta -= 0.8"""

                pos = np.concatenate((self.track.get_coordinate(rand_theta), self.track.get_angle(rand_theta)), axis=None)
                coeff = 2*random.random() - 1 # random in [-1, 1)
                width_vec = 0.5 * (np.array(self.track.get_coordinate(rand_theta, line='out'))-np.array(self.track.get_coordinate(rand_theta, line='int')))
                width = np.linalg.norm(width_vec)
                coeff = np.clip(coeff, -1+self.params['width']/width, 1-self.params['width']/width) # clips so car is correctly within track bounds
                pos[:2] += coeff*width_vec

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
        return normalisation.denorm_action(action, self.params)

    def get_orig_obs(self, obs, action, reward):
        """
        Takes a subset of the entire observation and flattens it for etter feeding into the RL network
        """
        obs = self.obs_preselection(obs)

        obs['scans'] = self.filter_scans(obs['scans'])

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

    def get_frenet_obs(self, obs, reward, action):
        obs = self.obs_preselection(obs)
        car_position = self.sim.agents[self.ego_idx].state[:2]
        car_angle = obs['poses_theta'][0]%(2*np.pi)

        # following line can be used if lidar but with less than 1080 scans is wanted
        filtered_scans = self.filter_scans(obs['scans'])
        
        new_obs = {'scans':np.array(filtered_scans)} #'scans': obs['scans']}
        #new_obs['param'] = [self.theta]
        new_obs['deviation'] = [np.linalg.norm(car_position-self.track.get_coordinate(self.theta))]
        rel_head = self.get_rel_head(obs)
        new_obs['rel_heading'] = [rel_head]
        new_obs['longitudinal_vel'] = [obs['linear_vels_x'][0]]
        new_obs['later_vel'] = [obs['linear_vels_y'][0]]
        new_obs['yaw_rate'] = [obs['ang_vels_z'][0]]

        # max_dev = self.get_max_dev()
        # if new_obs['deviation'][0] >= max_dev:
        #     reward = PENALTY

        keys = ['scans', 'deviation', 'rel_heading', 'longitudinal_vel', 'later_vel', 'yaw_rate']
        new_obs_norm = self.normalize_obs(new_obs)
        obs = np.concatenate([new_obs_norm[key] for key in keys], axis=None).astype(np.float32)

        # penalty onsteep action deviation
        reward -=  0.001*np.linalg.norm(self.prev_action - action)
        #print("Delta: {}".format(np.linalg.norm(self.prev_action - action)))
        self.prev_action = action

        return obs, reward
            
    def get_frenet_traj_obs(self, obs, reward, norm_action, action):
        """
        From the original observation, obtain the single vector observation
        and the reward, according to the trajectory-conditioned arch. 
        chosen 
        """

        info_obs = {'out_soft':False, 'bad_obs':False}
        
        ## PRE PROCESSING
        obs = self.obs_preselection(obs)
        
        filtered_scans = self.filter_scans(obs['scans'])
        car_position = self.sim.agents[self.ego_idx].state[:2]
        car_angle = obs['poses_theta'][0]%(2*np.pi)
        th_on_traj = self.track.trajectory.find_theta(car_position, self.theta+self.track.delta_traj)
        rel_head = self.get_rel_head(obs)
        slip_angle = self.sim.agents[0].state[6]
        rot_matr = np.array([[np.cos(car_angle), np.sin(car_angle)],[-np.sin(car_angle), np.cos(car_angle)]]) # rotates the points so that x axis is aligned with car
        trajectory = [rot_matr@(np.array(self.track.trajectory.get_coordinate(th))-car_position) for th in np.linspace(th_on_traj, th_on_traj+self._traj_len, num=self._traj_len)]
        
        new_obs = {'scans':np.array(filtered_scans)} 
        #new_obs['param'] = [self.theta]
        new_obs['deviation'] = [np.linalg.norm(car_position-self.track.trajectory.get_coordinate(th_on_traj))]
        new_obs['rel_heading'] = [rel_head]
        new_obs['longitudinal_vel'] = [obs['linear_vels_x'][0]]
        new_obs['later_vel'] = [obs['linear_vels_y'][0]]
        new_obs['yaw_rate'] = [obs['ang_vels_z'][0]]

        ## REWARD ADJUSTING
        
        # central_line_deviation = np.linalg.norm(car_position-self.track.get_coordinate(self.theta)) # different from deviation from trajectory!
        # if not self.constraints_satisfied(central_line_deviation, slip_angle):
        #     info_obs['out_soft'] = True
        #     reward = PENALTY
        
        logging.info("Base reward: {}".format(reward))
        deviation_penalty = new_obs['deviation'][0]*self._traj_coeff
        logging.info("Penalty for deviation: {}. Actual deviation: {}".format(deviation_penalty, new_obs['deviation'][0]))
        reward -= deviation_penalty   # a penalty on the deviation from the "predicted horizon"

        # penalty on steep action deviation
        # TODO consider removing as we do output regularization now
        # action_delta = np.linalg.norm(self.prev_action - action)
        # action_penalty = self._act_pen*action_delta
        # logging.info("Penalty for action delta: {}. Actual delta: {}".format(action_penalty, action_delta))
        # reward -= action_penalty

        ## NORMALISATION
        new_obs_norm = self.normalize_obs(new_obs)
        trajectory_norm = self.normalize_traj(trajectory)

        ## CONCATENATION
        if self.use_lidar:
            keys = ['scans', 'deviation', 'rel_heading', 'longitudinal_vel', 'later_vel', 'yaw_rate'] # hardcoded because getting them from dictionary does not imply correct ordering
        else:
            keys = ['deviation', 'rel_heading', 'longitudinal_vel', 'later_vel', 'yaw_rate'] # uncomment this line to remove the lidar
        new_obs_norm_vec = np.concatenate([new_obs_norm[key] for key in keys], axis=None).astype(np.float32)
        new_obs_final = np.concatenate((trajectory_norm, new_obs_norm_vec, norm_action), axis=None)
        
        # TODO why is this here??    
        self.prev_action = action

        return new_obs_final, reward, info_obs

    def normalize_obs(self, obs):
        return normalisation.normalise_observation(obs, self.params)

    def normalize_traj(self, trajectory):
        return normalisation.normalise_trajectory(trajectory, self._traj_len)

    def constraints_satisfied(self, cl_deviation, slip_angle):
        maximum_deviation = self.get_max_dev()

        if cl_deviation > maximum_deviation:
            return False
        
        if np.abs(slip_angle) > self._angle_limit:
            return False

        return True 

    def filter_scans(self, scans):
        """
        Filters the lidar scans, reducing the dimention to only

        # TODO improve overall (average instead of picking, param the angle)
        """
        return [scans[180+4*18*i] for i in range(11)]

    def obs_preselection(self, obs):
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

    def get_car_state(self, idx:int=0):
        return self.sim.agents[idx].state

    def get_car_advancement(self, idx: int= 0):
        # retunrns the completed percentage of the track
        return self.theta/self.track.track_length

    def get_car_squiggliness(self, idx:int=0):
        track_angle = self.track.get_angle(self.theta)
        return np.abs(self.sim.agents[0].state[2]%(2*np.pi) - track_angle%(2*np.pi))

    def update_max_vel(self, max_vel):
        """
        Function to update the maximum velocty, be careful at it changes
        the input to the system as the action is normalised. 
        Ideally is only used for curriculum RL. 
        """
        self.params['v_max'] = max_vel

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


class RLF110EnvDirect(RLF110Env):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.sim = Simulator(self.params, self.num_agents, self.seed, direct_input=True, slow_threshold=self.slow_threshold)
        self.sim.set_map(self.map_path, self.map_ext)

    def denorm_action(self, action):
        """
        As the action is by default clipped within -1, 1, we need to rescale it to the proper size
        Here we are using the direct acceleration and steering velocity though
        """
        return normalisation.denorm_action_direct(action, self.params)
