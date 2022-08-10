import os
import time
import yaml
import pathlib

import numpy as np
from argparse import Namespace

from stable_baselines3 import SAC
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecVideoRecorder


dir_path = pathlib.Path(__file__).parent.resolve()
model_dir = os.path.join(dir_path, 'models')
configs_dir = os.path.join(dir_path, 'configs')


def test(env_conf):
    """
    Tester for Reinforcement Learning algorithms
    """
   
    env = make_vec_env(
        env_id='f110_gym:f110rl-v0', 
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
            'model_{}_SAC_{}_noise_{}/wandb_models/model'.format(
                env_conf['map_name'],
                env_conf['obs_type'], 
                str(env_conf['params_noise'])
                )
            ), 
        env=env,
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
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
        conf = Namespace(**conf_dict)

    env_conf = {'map': os.path.join(configs_dir, '{}'.format(map_name)),
                    'map_ext': conf.map_ext,
                    'map_name': map_name,
                    'random_init': True,
                    'sx': conf.sx,
                    'sy': conf.sy,
                    'stheta': conf.stheta,
                    'num_agents': 1,
                    'ep_len':ep_len,
                    'obs_type':'Frenet_trajectory',
                    'params_noise': params_noise, 
                    'var_mu':(0.075/2)**2,
                    'var_Csf':0,
                    'var_Csr':0,
                    'redraw_upon_reset':True,
                    'angle_limit':ang_deg*np.pi/180, # 30 deg
                    'use_trajectory': use_trajectory,
                    }

    return env_conf

if __name__ == '__main__':
    map_name = 'f'
    env_conf = init_confs(
            map_name, 
            ep_len=10000, 
            params_noise=False
        )

    test(env_conf)
    