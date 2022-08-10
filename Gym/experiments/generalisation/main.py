###########
# Script to run the experiments to test generalisation 
# of the RL models
###########

### IMPORTS 
import os, yaml, pathlib
import gym
import numpy as np
import wandb
from argparse import Namespace

from stable_baselines3 import SAC


### PATHS
dir_path = pathlib.Path(__file__).parent.parent.parent.resolve()
configs_dir = os.path.join(dir_path, 'configs')
model_dir = os.path.join(dir_path, 'models')


### HELPER FUNCTIONS
def init_confs(map_name: str, obs_type: str):
    with open(os.path.join(configs_dir, '{}.yaml'.format(map_name))) as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
        conf = Namespace(**conf_dict)

    env_conf = {'map': os.path.join(configs_dir, '{}'.format(map_name)),
                    'map_ext': conf.map_ext,
                    'random_init': False,
                    'sx': conf.sx,
                    'sy': conf.sy,
                    'stheta': conf.stheta,
                    'num_agents': 1,
                    'ep_len':10000, # this sets the maximum episode length, it is ~1.5 times the best time for a lap, so it changes from track to track
                    'obs_type':obs_type,
                    }

    return env_conf


### MAIN CODE
def main(map_name, obs_type, noise):
    # INITS
    laptimes = np.zeros((5,1))
    crashed = np.zeros((5,1))
    advancement = np.zeros((5,1))
    run = wandb.init("trajectory_obtainer", project="forl", entity="forzapbl")



    #for i in range(5):
    for i in [3]:
        # LOOP INITS
        env_conf = init_confs(map_name, obs_type)
        env = gym.make(id='f110_gym:f110rl-v0', **env_conf)
        env.training = False
        obs = env.reset()
        done = False
        model_path = os.path.join(model_dir, 'model_SOCHI_SAC_{}_noise_{}/best_model'.format(obs_type, noise))
        agent = SAC.load(path=model_path, env=env, device="auto")
        laptime = 0

        # SIMULATION
        while not done:
            action, _ = agent.predict(observation=obs, deterministic=True)
            obs, _, done, info = env.step(action=action)
            laptime += 0.01
            wandb_log={}
            state = env.get_car_state()
            wandb_log['position_x'] = state[0]
            wandb_log['position_y'] = state[1]
            wandb.log(wandb_log)
            #env.render()

        laptimes[i] = laptime
        crashed[i] = not info['checkpoint_done']
        advancement[i] = env.get_car_advancement()

    with open('./results/{}_{}_noise_{}_results.txt'.format(map_name, obs_type, noise), 'w') as file:
        file.writelines("Laptimes for the five models: \n")
        file.writelines("\n".join([str(lt) for lt in laptimes]))
        file.writelines("\n\n\n")
        file.writelines("Has the car crashed? : \n")
        file.writelines("\n".join([str(cr) for cr in crashed]))
        file.writelines("\n\n\n")
        file.writelines("Percentual of advancement : \n")
        file.writelines("\n".join([str(perc) for perc in advancement]))


if __name__ == '__main__':
    valid_maps = ['circle', 'squarish', 'f']
    valid_maps = ['SOCHI']
    obs_types = ['Frenet_trajectory', 'original']
    obs_types = ['Frenet_trajectory']
    noises = [True, False]
    noises = [False]

    for map_name in valid_maps:
        for obs_type in obs_types:
            for noise in noises:
                main(map_name=map_name, obs_type=obs_type, noise=noise)

    
