import os
import time
import yaml
import pathlib

import numpy as np
from stable_baselines3 import SAC
from f110_gym.envs import normalisation
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecVideoRecorder


# paths
dir_path = pathlib.Path(__file__).resolve().parents[1]
configs_dir = os.path.join(dir_path, "cfg")
model_dir = os.path.join(dir_path, "models")


def test(env_conf):
    """
    Tester for TC-Driver
    """
    env = make_vec_env(
        env_id=env_conf['env_id'], 
        n_envs=1, 
        env_kwargs=env_conf,
        )
    env.training = False

    env_rec = VecVideoRecorder(
        env, 
        "recording",
        record_video_trigger=lambda x: True, 
        video_length=2000,
        )

    model = SAC.load(
        os.path.join(
            model_dir, 
            f'model_f_SAC_{env_conf["obs_type"]}_noise_{env_conf["params_noise"]}/model_last'
            ), 
        env=env,
        device='cpu',
        )

    done = False
    laptime = 0.0
    start = time.perf_counter()
    obs = env_rec.reset()
    while not done:
        action, _states = model.predict(obs, deterministic=True)
        obs, step_reward, done, info = env_rec.step(action)
        # print('Reward: ', step_reward)
        # print('Action: ', action)
        # print(obs)
        laptime += 0.01
        env_rec.render(mode='human_fast')
    print(
        'Sim elapsed time: {:.2f} Real elapsed time: {:.2f}'.format(
            laptime, time.perf_counter() - start
            )
        )

def init_confs(
    map_name, 
    ep_len: int = 500, 
    params_noise: bool = True, 
    ang_deg: int = 30,
    use_trajectory: bool = True,
    ):
    
    with open(os.path.join(configs_dir, '{}.yaml'.format(map_name))) as file:
        conf_dict = yaml.safe_load(file)

    with open(os.path.join(configs_dir, "car_params.yaml".format(map))) as file:
        params = yaml.safe_load(file)

    env_conf = {'map': os.path.join(configs_dir, '{}'.format(map_name)),
                    'map_ext': conf_dict['map_ext'],
                    'map_name': map_name,
                    'random_init': False,
                    'sx': conf_dict['sx'],
                    'sy': conf_dict['sy'],
                    'stheta': conf_dict['stheta'],
                    'num_agents': 1,
                    'ep_len':ep_len,
                    'obs_type':'Frenet',
                    'params_noise': params_noise, 
                    'var_mu':(0.075/2)**2,
                    'var_Csf':0,
                    'var_Csr':0,
                    'redraw_upon_reset':True,
                    'angle_limit':ang_deg*np.pi/180, # 30 deg
                    'use_trajectory': use_trajectory,
                    'max_vel':5,
                    'env_id':'f110_gym:f110rl-v0',
                    'params':params
                    }

    return env_conf

if __name__ == '__main__':
    map_name = 'f'
    env_conf = init_confs(
            map_name, 
            ep_len=10000, 
            params_noise=True
        )

    test(env_conf)
    