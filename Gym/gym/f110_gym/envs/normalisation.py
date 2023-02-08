"""
Normalization module for the RL environment
"""
import numpy as np

def denorm_action(action, params):
        """
        As the action is by default clipped within -1, 1, we need to rescale it to the proper size
        """

        v_min = params['v_min']
        v_max = params['v_max']

        if action.shape == (2,):
            action[0] = ((action[0] + 1)/2)*(params['s_max'] - params['s_min']) + params['s_min']
            action[1] = ((action[1] + 1)/2)*(v_max - v_min) + v_min
        elif action.shape == (1, 2):
            action[0][0] = ((action[0][0] + 1)/2)*(params['s_max'] - params['s_min']) + params['s_min']
            action[0][1] = ((action[0][1] + 1)/2)*(v_max - v_min) + v_min
        else:
            action[0][0] = ((action[0][0] + 1)/2)*(params['s_max'] - params['s_min']) + params['s_min']
            action[1][0] = ((action[1][0] + 1)/2)*(v_max - v_min) + v_min

        return action

def denorm_action_direct(action, params):
        """
        As the action is by default clipped within -1, 1, we need to rescale it to the proper size
        Here we are using the direct acceleration and steering velocity though
        """

        a_max = params['a_max']
        sv_max = params['sv_max']

        if action.shape == (2,):
            action[0] = ((action[0] + 1)/2)*(sv_max - (-sv_max)) + (-sv_max)
            action[1] = ((action[1] + 1)/2)*(a_max - (-a_max)) + (-a_max)
        elif action.shape == (1, 2):
            action[0][0] = ((action[0][0] + 1)/2)*(sv_max - (-sv_max)) + (-sv_max)
            action[0][1] = ((action[0][1] + 1)/2)*(a_max - (-a_max)) + (-a_max)
        else:
            action[0][0] = ((action[0][0] + 1)/2)*(sv_max - (-sv_max)) + (-sv_max)
            action[1][0] = ((action[1][0] + 1)/2)*(a_max - (-a_max)) + (-a_max)

        return action

def normalise_observation(obs, params, with_lidar=True):
    """
    Normalises the base observation.

    Args:
        obs: the base (Frenet) observation. It is a dictionary with the following keys: 
            scans: a list/np.array of lidar scans
            deviation: a scalr distance from a reference trajectory
            rel_heading: the scalar relative yaw to th e reference trajectory
            longitudinal_vel: the scalar longitudinal velocity of the car
            lateral_vel: the scalar lateral velocity of the car

        params: a dictionary containing the upper and lower limits for the different states of the observation. 
            Currently only contains: 
            v_min: minimum velocity
            v_max: maximum velocity
            width: width of the car
    """
    
    max_scan = 30
    min_scan = 0
    if with_lidar:
        obs['scans'] = np.clip(obs['scans'], min_scan, max_scan)
        obs['scans'] -= (max_scan-min_scan)/2
        obs['scans'] *= 2/(max_scan - min_scan)

    max_deviation = 10*params['width']
    min_deviation = 0
    obs['deviation'] = np.clip(obs['deviation'], min_deviation, max_deviation)
    obs['deviation'] -= (max_deviation-min_deviation)/2
    obs['deviation'] *= 2/(max_deviation - min_deviation)

    max_rel_heading = np.pi
    min_rel_heading = -np.pi
    obs['rel_heading'] = np.clip(obs['rel_heading'], min_rel_heading, max_rel_heading)
    obs['rel_heading'] -= (max_rel_heading-min_rel_heading)/2
    obs['rel_heading'] *= 2/(max_rel_heading - min_rel_heading)

    max_longitudinal_vel = params['v_max']
    min_longitudinal_vel = params['v_min']
    obs['longitudinal_vel'] = np.clip(obs['longitudinal_vel'], min_longitudinal_vel, max_longitudinal_vel)
    obs['longitudinal_vel'] -= (max_longitudinal_vel-min_longitudinal_vel)/2
    obs['longitudinal_vel'] *= 2/(max_longitudinal_vel - min_longitudinal_vel)

    max_later_vel = params['v_max']
    min_later_vel = params['v_min']
    obs['later_vel'] = np.clip(obs['later_vel'], min_later_vel, max_later_vel)
    obs['later_vel'] -= (max_later_vel-min_later_vel)/2
    obs['later_vel'] *= 2/(max_later_vel - min_later_vel)

    max_yaw_rate = 3.2 
    min_yaw_rate = -3.2
    obs['yaw_rate'] = np.clip(obs['yaw_rate'], min_yaw_rate, max_yaw_rate)
    obs['yaw_rate'] -= (max_yaw_rate-min_yaw_rate)/2
    obs['yaw_rate'] *= 2/(max_yaw_rate - min_yaw_rate)

    return obs

def normalise_trajectory(traj, traj_len):
    """
    Normalises the trajectory according to the trajectory length. 
    """
    max_traj = traj_len
    min_traj = -traj_len
    trajectory = np.clip(traj, min_traj, max_traj)
    trajectory -= (max_traj-min_traj)/2
    trajectory *= 2/(max_traj - min_traj)

    return trajectory
