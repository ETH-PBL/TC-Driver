import os
import pathlib

import time
import tqdm
import yaml
import numpy as np
import pandas as pd
from stable_baselines3 import SAC
from splinify.splinify import SplineTrack
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecVideoRecorder

import cl_parser
from data_gatherer import DataGatherer




def main(sim_conf: dict, car_params):
    """
    Data gathering script for RL agents
    """
    cur_dir = pathlib.Path(__file__).resolve().parent
    root_dir = pathlib.Path(__file__).resolve().parents[2]
    model_dir = os.path.join(root_dir, sim_conf['model_location'])

    car_params_list = [dict() for _ in range(sim_conf['n_exp'])]
    episode_endings = ["" for _ in range(sim_conf['n_exp'])]
    advancement = [0 for _ in range(sim_conf['n_exp'])]
    sim_conf2 = sim_conf.copy()
    sim_conf2['params'] = car_params

    # 1 LOAD ENVIRONMENT AND AGENT #
    ################################
    print(sim_conf)
    env = make_vec_env(
        env_id=sim_conf['env_id'], 
        n_envs=1, 
        env_kwargs=sim_conf2,
        )
    env.training = False

    driver = SAC.load(
        os.path.join(
            model_dir, 
            sim_conf['model_choice']
            ), 
        env=env,
        device='cpu',
    )
    data_saver = DataGatherer(max_eplen=sim_conf['ep_len'], n_exp=sim_conf['n_exp'])

    # 2 RUN SIMULATIONS #
    #####################

    for idx_exp in tqdm.tqdm(range(sim_conf['n_exp'])):
        done = False
        laptime = 0.0
        start = time.perf_counter()
        obs = env.reset()
        i = 0
        car_params_list[idx_exp] = env.get_attr('sim', 0)[0].agents[0].params.copy()
        while not done:
            # gather data
            sim, = env.get_attr('sim', 0)
            data_saver.data['s_x'][i,idx_exp] = sim.agents[0].state[0]
            data_saver.data['s_y'][i,idx_exp] = sim.agents[0].state[1]
            data_saver.data['velocity'][i,idx_exp] = sim.agents[0].state[3]
            #data_saver.data['dist_traj'][i,idx_exp] = sim.agents[0].state[] # TODO
            data_saver.data['yaw'][i,idx_exp] = sim.agents[0].state[4]
                        
            # simulate
            action, _states = driver.predict(obs, deterministic=True)
            obs, step_reward, done, info = env.step(action)
            # env.render(mode='human')
            laptime += 0.01
            i += 1
        print(
            'Sim elapsed time: {:.2f} Real elapsed time: {:.2f}'.format(
                laptime, time.perf_counter() - start
                )
            )
        episode_endings[idx_exp], = env.get_attr('reason', 0)
        advancement[idx_exp] = str(env.get_attr('last_tot_adv', 0)[0])

    # 3 SAVE DATA #
    ###############
    experiment_name = f'{sim_conf["obs_type"]}_{sim_conf["map_name"]}'
    exp_dir = os.path.join(cur_dir, '{}/'.format(experiment_name))
    # if folder does not exist, create it
    if not os.path.isdir(exp_dir):
        os.mkdir(exp_dir)
    # save data 
    data_saver.save_data(exp_dir)

    # save configs
    with open(f"{exp_dir}sim_conf.txt", "w") as file:
        yaml.safe_dump(sim_conf, file)

    car_params_df = pd.DataFrame(index=car_params_list[0])
    for i, el in enumerate(car_params_list):
        car_params_df[f"car_params_{i}"] = el.values()
    car_params_df.to_csv(f"{exp_dir}car_params.csv")

    with open(f"{exp_dir}episode_endings.csv", "w") as file:
        file.writelines(",\n".join(episode_endings))
    
    with open(f"{exp_dir}advancement.csv", "w") as file:
        file.writelines(",\n".join(advancement))

def init_conf(file_name):
    root_dir = pathlib.Path(__file__).resolve().parents[2]
    config_dir = os.path.join(root_dir, 'cfg')

    # simulation conf
    try:
        with open(config_dir + "/sim/" + file_name) as file:
            conf_file = yaml.safe_load(file)
        conf_file['map'] = os.path.join(config_dir, conf_file['map_name'])
    except FileNotFoundError as e: 
        raise e #FileNotFoundError(f"Configuration file {file_name} is not existing or at another location")
    
    # car conf
    with open(os.path.join(config_dir, conf_file['car_conf_file'])) as file:
        car_params = yaml.safe_load(file)

    return conf_file, car_params


if __name__ == '__main__':
    # parse command line arguments
    parser = cl_parser.init_parser()
    args = parser.parse_args()

    # initialize configuration dictionary
    file_name = args.file_name
    sim_conf, car_params = init_conf(file_name)

    # run the simulation
    main(sim_conf, car_params)
