import os, time, yaml, logging, pathlib
from argparse import Namespace

import gym
import wandb
import numpy as np
from gym.wrappers import NormalizeReward
from gym.wrappers.rescale_action import RescaleAction
from wandb.integration.sb3 import WandbCallback
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.envs.multi_input_envs import SimpleMultiObsEnv
from stable_baselines3.common.vec_env import DummyVecEnv, VecVideoRecorder, VecNormalize
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback

from utils.utils import NormalizedCheckpointCallback


#PATH Defines
dir_path = pathlib.Path(__file__).parent.resolve()
configs_dir = os.path.join(dir_path, 'configs')
model_dir = os.path.join(dir_path, 'models')

#AVAILABLE modes = ['end2end', 'hierarchical'] maps = ['circle', 'SOCHI'] Architectures = ['PPO', 'SAC']
sb3_ppo_config = {'policy_type': 'MlpPolicy', 'total_timesteps': 1e6, 'gamma': 0.99}


def train(exp_conf, continue_training=False):
    env_conf = init_confs(exp_conf)

    env = make_vec_env(env_id='f110_gym:f110rl-v0', n_envs=1, env_kwargs=env_conf)

    env = VecVideoRecorder(env, "recording",
                           record_video_trigger=lambda x: x % int(wandb.config.total_timesteps / 10) == 0,
                           video_length=500)
    env.reset()

    #Wandb init for logging
    run = wandb.init(name="{}_{}_{}_noise_{}_{}".format(env_conf['obs_type'], exp_conf['arch'], exp_conf['map'], str(env_conf['params_noise']), int(time.time())),
                     project="forl",
                     entity="forzapbl",
                     config=exp_conf['conf'],
                     sync_tensorboard=True,  # auto-upload sb3's tensorboard metrics
                     monitor_gym=True,  # auto-upload the videos of agents playing the game
                     save_code=True,
                     mode="run") # set to `disabled` to make wandb not run, set it to `run` to enable wandb
    wandb.run.log_code(".")

    #Choose if you want to finetune a pretrained model pbl_f110_gym/models/model_SAC/periodic_save/rl_model_100000_steps.zip
    if continue_training:
        model = SAC.load(os.path.join(model_dir, 'model_SOCHI_SAC_{}_noise_{}/best_model.zip'.format(env_conf['obs_type'], str(env_conf['params_noise']))),
                         env=env,
                         verbose=1,
                         gamma=wandb.config.gamma,
                         use_sde=False,
                         tensorboard_log="runs/{}".format(run.name))
    else:
        model = SAC(wandb.config.policy_type, env, verbose=1, tensorboard_log="runs/{}".format(run.name), batch_size=64, device='auto', train_freq=1)

    # callbacks
    wandb_callback = WandbCallback(gradient_save_freq=100,
                                       model_save_freq=int(wandb.config.total_timesteps / 10),
                                       model_save_path=os.path.join(model_dir, 'model_SOCHI_SAC_{}_noise_{}/wandb_models/'.format(env_conf['obs_type'], str(env_conf['params_noise']))))

    eval_env = make_vec_env(env_id='f110_gym:f110rl-v0', n_envs=1, env_kwargs=env_conf, wrapper_class=RescaleAction, wrapper_kwargs={'min_action':-1, 'max_action':1})
    eval_env = VecVideoRecorder(eval_env, "recording",
                           record_video_trigger=lambda x: x % int(wandb.config.total_timesteps / 10) == 0,
                           video_length=500)
    best_model_callback = EvalCallback(eval_env, 
                             best_model_save_path=os.path.join(model_dir, 'model_SOCHI_SAC_{}_noise_{}/'.format(env_conf['obs_type'], str(env_conf['params_noise']))),
                             log_path=os.path.join(model_dir, 'model_SOCHI_SAC_{}_noise_{}/logs/'.format(env_conf['obs_type'], str(env_conf['params_noise']))),
                             eval_freq=wandb.config.total_timesteps/(10),
                             deterministic=True, render=False)


    #Actual learning going on
    model.learn(total_timesteps=wandb.config.total_timesteps,
                eval_log_path=os.path.join(dir_path, 'train_logs'),
                callback=[wandb_callback, best_model_callback])

    run.finish()

def test(exp_conf, map_name, mode):
    env_conf = init_confs(exp_conf)
   
    env = make_vec_env(env_id='f110_gym:f110rl-v0', n_envs=1, env_kwargs=env_conf)
    env.training = False

    env_rec = VecVideoRecorder(env, "recording",
                           record_video_trigger=lambda x: True, video_length=2000)

    model = SAC.load(os.path.join(model_dir, 'model_SOCHI_SAC_{}_noise_{}/best_model.zip'.format(env_conf['obs_type'], str(env_conf['params_noise']))), env=env)

    done = False
    laptime = 0.0
    start = time.perf_counter()
    obs = env_rec.reset()
    while not done:
        action, _states = model.predict(obs, deterministic=True)
        obs, step_reward, done, info = env_rec.step(action)
        #print('Reward: ', step_reward)
        #print('Action: ', action)
        #print(obs)
        laptime += 0.01
        env_rec.render(mode='human_fast')
    print('Sim elapsed time: {:.2f} Real elapsed time: {:.2f}'.format(laptime, time.perf_counter() - start))

def init_confs(exp_conf):
    
    with open(os.path.join(configs_dir, '{}.yaml'.format(exp_conf['map']))) as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
        conf = Namespace(**conf_dict)

    env_conf = {'map': os.path.join(configs_dir, '{}'.format(exp_conf['map'])),
                    'map_ext': conf.map_ext,
                    'random_init': True,
                    'sx': conf.sx,
                    'sy': conf.sy,
                    'stheta': conf.stheta,
                    'num_agents': 1,
                    'ep_len':10000, # this sets the maximum episode length, it is ~1.5 times the best time for a lap, so it changes from track to track
                    'obs_type':'Frenet_trajectory',
                    'params_noise':False, 
                    'var_mu':(0.075/2)**2, # if all are set to 0 no noise is applied
                    'var_Csf':0,
                    'var_Csr':0,
                    'redraw_upon_reset':True}

    return env_conf

if __name__ == '__main__':
    exp_conf = {'mode': 'Frenet_trajectory', 'map': 'SOCHI', 'arch': 'SAC', 'conf': sb3_ppo_config}
    logging.basicConfig(filename=os.path.join(dir_path, 'logs/last_run.log'), level=logging.INFO)
    
    run_mode = 'train'

    logging.info("Starting a {}\n".format(run_mode))
    if run_mode == 'train':
        train(exp_conf=exp_conf, continue_training=False)
    elif run_mode == 'test':
        test(exp_conf=exp_conf, map_name='SOCHI', mode='SAC')
    logging.info("{} finished.\n".format(run_mode))
