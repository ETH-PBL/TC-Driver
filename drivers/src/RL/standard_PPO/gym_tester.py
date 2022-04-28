import gym
from gym.wrappers import Monitor, NormalizeReward, RescaleAction
from argparse import Namespace
import numpy as np
import yaml, time, pathlib, os
from stable_baselines3 import PPO
from stable_baselines3.common.envs.multi_input_envs import SimpleMultiObsEnv
from stable_baselines3.common.callbacks import EvalCallback

dir_path = pathlib.Path(__file__).parent.parent.resolve()
configs_dir = os.path.join(dir_path, 'configs')

model_path = pathlib.Path(__file__).parent.resolve()
model_dir = 'logs/best_model'

continue_training = False


def testo(map_name):
    with open(os.path.join(configs_dir, '{}.yaml'.format(map_name))) as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
    conf = Namespace(**conf_dict)
    env = gym.make('f110_gym:f110rl-v0',
                   map=os.path.join(configs_dir, '{}'.format(map_name)),
                   map_ext=conf.map_ext,
                   sx=conf.sx,
                   sy=conf.sy,
                   stheta=conf.stheta,
                   num_agents=1, 
                   random_init=True,
                   params_noise=True,
                   var_Csf=0.01,
                   var_Csr=0.01,
                   var_mu=0.01,
                   redraw_upon_reset=True, 
                   ep_len = 3000,
                   obs_type = "Frenet")
    env = NormalizeReward(env)
    env = RescaleAction(env, -1, 1)
    print(env.observation_space)
    
    eval_env = gym.make('f110_gym:f110rl-v0',
                   map=os.path.join(configs_dir, '{}'.format(map_name)),
                   map_ext=conf.map_ext,
                   sx=conf.sx,
                   sy=conf.sy,
                   stheta=conf.stheta,
                   num_agents=1)
    eval_env = Monitor(eval_env, os.path.join(model_path, 'logs/recordings'), video_callable=False, force = True)
    eval_env = RescaleAction(eval_env, -1, 1)
    eval_callback = EvalCallback(eval_env, best_model_save_path=os.path.join(model_path, 'logs/'),
                             log_path=os.path.join(model_path, 'logs/'),
                             eval_freq=1e5,
                             deterministic=True, render=False)
    
    env_rec = Monitor(env, os.path.join(model_path, 'logs/recordings'), force=True)
    env.reset()

    if continue_training:
        model = PPO.load(os.path.join(model_path, model_dir), env=env, verbose=1, gamma=0.99999, use_sde=True)
    else:
        model = PPO("MultiInputPolicy", env, verbose=1, gamma=0.99999, use_sde=True)
    model.learn(total_timesteps=10, eval_log_path=os.path.join(dir_path, 'train_logs'), callback=eval_callback)
    model.save(path=os.path.join(model_path, 'models/model'))

    done = False
    laptime = 0.0
    start = time.time()

    obs = env_rec.reset()
    while not done:
        action, _states = model.predict(obs, deterministic=True)
        obs, step_reward, done, info = env_rec.step(action)
        print(obs)
        #print('Reward: ', step_reward)
        #print('Action: ', action)
        laptime += step_reward
        env_rec.render(mode='human_fast')
    print('Sim elapsed time:', laptime, 'Real elapsed time:', time.time() - start)





if __name__ == '__main__':
    map_name = 'circle'
    testo(map_name=map_name)
