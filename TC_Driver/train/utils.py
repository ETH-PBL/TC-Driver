import logging
import os
import time
import pathlib

import wandb
import numpy as np
import pandas as pd
from moviepy.editor import ImageSequenceClip
from stable_baselines3.common.callbacks import BaseCallback

from splinify.splinify import SplineTrack
from index import Index


root_dir = pathlib.Path(__file__).parents[1].resolve()
tracks_dir = os.path.join(root_dir, 'cfg/')
recording_dir = os.path.join(root_dir, 'train/recording/')

class MetricEvalCallback(BaseCallback):
    """
    Callback that automatically evaluates the metrics and stores them to
    weights and biases.
    It also records a video of said metrics evaluation.
    """

    def __init__(self, eval_env, eval_freq, wandb_run, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.eval_env = eval_env
        self.eval_freq = eval_freq
        self.wandb_run = wandb_run


    def _on_step(self) -> bool:

        super()._on_step()
        
        # log data
        if self.eval_freq > 0 and self.n_calls % self.eval_freq == 0:
            # evaluate metric in 
            wandb_log = evaluate_metrics(
                self.eval_env, 
                self.model, 
                max_tsteps=self.eval_env.env_method("get_ep_len")[0], 
                track_shrink_coeff=0.31*1.5,
                idx=self.n_calls//self.eval_freq
                )
        
            self.wandb_run.log(wandb_log)
            
        return True

def evaluate_metrics(
    env, 
    agent, 
    max_tsteps, 
    track_shrink_coeff, 
    idx,
    track_name: str = 'f', 
    start_par: int = 0,
    ):
    print("Evaluating metrics")

    done = False
    track_path = os.path.join(tracks_dir, track_name)
    track = SplineTrack(str(track_path) + '_waypoints.txt', safety_margin=track_shrink_coeff)
    start_point = track.get_coordinate(start_par)
    start_angle = track.get_angle(start_par)
    start_pose = np.concatenate((start_point, [start_angle]), axis = 0).reshape(1,-1)
    theta = start_par
    obs = env.reset()

    states = np.empty((max_tsteps+1, 7), dtype=np.float32)
    actions = np.empty((max_tsteps+1, 2), dtype=np.float32)
    advancements = np.empty((max_tsteps+1, 1), dtype=np.float32)
    computation_times = np.empty((max_tsteps+1, 1), dtype=np.float32)
    flags = np.empty((max_tsteps+1, 2), dtype=np.int8) # lap_finished, out_of_soft_boundaries
    columns_names = [
        's_x', 's_y', 'delta', 'velocity', 'yaw', 'yaw_der', 'slip_angle',
        'steer_v', 'acceleration',
        'param', 
        'comp_time', 
        'lap_finished', 'out_of_soft',
        ]
    done = False
    info = [{}]
    info[0]['checkpoint_done'] = 0
    video_list = []
    i = 0
    
    print("Gathering data")
    while not done:
        state = env.get_attr('sim')[0].agents[0].state
        states[i, :] = state

        theta = track.find_theta(state[:Index.S_Y+1], theta)
        flag_finished = info[0]['checkpoint_done']
        flag_oo_soft = check_oo_soft(track, state, theta)
        advancements[i] = theta
        flags[i, :] = [flag_finished, flag_oo_soft]
        theta += 0.01*state[Index.V]
        
        time1 = time.perf_counter()
        action, _ = agent.predict(obs)
        time2 = time.perf_counter()
        actions[i, :] = action
        computation_times[i] = time2-time1

        obs, _, done, info = env.step(np.array([action]))
        video_list.append(env.render(mode='rgb_array'))
        i += 1
    
    # saving video
    video = ImageSequenceClip(video_list, fps=100)
    videofilename = f"{recording_dir}evaluation_rec_{int(idx):02}.mp4"
    video.write_videofile(videofilename, audio=False, codec="libx264")
    logging.info(f"Metrics evaluation video saved with name: '{videofilename}'")

    
    all_data = np.concatenate((states, actions, advancements, computation_times, flags), axis=1)
    all_data = all_data[:i, :]
    df = pd.DataFrame(all_data, columns=columns_names) # TODO save csv so wandb logs it 
    
    wandb_log = {}
    # wandb_log['full_df_{}'.format(idx)] = wandb.Table(dataframe=df)
    wandb_log['eval_advancement'] = df['param'].iloc[-1]
    wandb_log['soft_constraints_viol'] = df['out_of_soft'].sum()/(i+1)
    wandb_log['lap_time'] = (i+1)/100
    wandb_log['lap_finished'] = info[0]['checkpoint_done']

    df['vx'] = df['velocity']*np.cos(df['slip_angle'] + df['yaw'])
    df['vy'] = df['velocity']*np.sin(df['slip_angle'] + df['yaw'])
    df['acc_rel_x'] = df['vx'].diff().fillna(0)/(0.01)
    df['acc_rel_y'] = df['vy'].diff().fillna(0)/(0.01)
    wandb_log['acc_on_car_{}'.format(idx)] = wandb.Table(dataframe=df[['acc_rel_y', 'acc_rel_x']]) # TODO fix formatting of table (prob no can do)
    wandb_log['video'] = wandb.Video(videofilename)

    return wandb_log


def check_oo_soft(track: SplineTrack, state, theta):
    """
    Checks if the car is out of the soft constraints
    """

    coord = np.array(state[:Index.S_Y+1])
    centr_coord = np.array(track.get_coordinate(theta, line='mid'))
    left_coord = np.array(track.get_coordinate(theta, line='int'))
    right_coord = np.array(track.get_coordinate(theta, line='out'))

    dist_left = np.linalg.norm(left_coord-coord)
    dist_right = np.linalg.norm(right_coord-coord)

    dist_car_c = np.linalg.norm(coord - centr_coord)
    if dist_left < dist_right:
        dist_bound_c = np.linalg.norm(left_coord - centr_coord)        
    else:
        dist_bound_c = np.linalg.norm(right_coord - centr_coord)

    return int(dist_car_c > dist_bound_c)
