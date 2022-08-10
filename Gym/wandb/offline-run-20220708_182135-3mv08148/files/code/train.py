import os, time, yaml, logging, pathlib
import argparse
from argparse import Namespace

import gym
import wandb
import numpy as np
from gym.wrappers import NormalizeReward
from wandb.integration.sb3 import WandbCallback
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.vec_env import DummyVecEnv, VecVideoRecorder
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback, BaseCallback
from stable_baselines3.common.utils import safe_mean

from utils.callbacks import CurriculumVelCallback, MetricEvalCallback


# PATH Defines
dir_path = pathlib.Path(__file__).parent.resolve()
configs_dir = os.path.join(dir_path, "configs")
model_dir = os.path.join(dir_path, "models")

# AVAILABLE modes = ['end2end', 'hierarchical'] maps = ['circle', 'SOCHI'] Architectures = ['PPO', 'SAC']
sb3_ppo_config = {"policy_type": "MlpPolicy", "total_timesteps": 1e6, "gamma": 0.99}

class CurrSAC(SAC):
    """
    Very quick wrapper to introduce velocity curriculum
    """
    def _on_step(self):
        rew = safe_mean([ep_info["r"] for ep_info in self.ep_info_buffer])
        len = safe_mean([ep_info["l"] for ep_info in self.ep_info_buffer])
        cur_mv = self.env.get_attr("params")[0]['v_max']
        cur_vel = rew/(0.01*len)
        if cur_vel >= cur_mv*0.9 and len >= 420/(cur_vel*0.01):
            new_mvel = cur_mv*1.02 # TODO hardcoded param for curriculum
            new_mvel = np.clip(new_mvel, 0.1, 20) # TODO hardcoded velocity bounds
            self.env.env_method("update_max_vel", new_mvel)

class CurrCallback(BaseCallback):
    def __init__(self, wandb_run, eval_freq, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        self.eval_freq = eval_freq
        self.wandb_run = wandb_run

    def _on_step(self):
        super()._on_step()

        if self.eval_freq > 0 and self.n_calls % self.eval_freq == 0:
            wandb_log = {
                "curriculum_velocity": self.training_env.get_attr("params")[0]['v_max']
            }
            self.wandb_run.log(wandb_log)

def train(exp_conf, env_conf, continue_training=False):
    model_name = "model_{}_SAC_{}_noise_{}".format(
        exp_conf["map"], env_conf["obs_type"], str(env_conf["params_noise"])
    )

    if sb3_ppo_config["total_timesteps"] == 1e6:
        project = "forl"
    elif sb3_ppo_config["total_timesteps"] == 3e6:
        project = "long_runs"
    else:
        project = "forl"
    
    # Wandb init for logging
    run = wandb.init(
        name="{}_{}_{}_noise_{}_{}".format(
            env_conf["obs_type"],
            exp_conf["arch"],
            exp_conf["map"],
            str(env_conf["params_noise"]),
            int(time.time()),
        ),
        project=project,
        entity="forzapbl",
        config=env_conf,
        sync_tensorboard=True,  # auto-upload sb3's tensorboard metrics
        monitor_gym=True,  # auto-upload the videos of agents playing the game
        save_code=True,
        mode="offline", # can be `disabled`, `offline`, `run`
    )  
    wandb.run.log_code(".")
    try:
        env_conf["traj_coeff"] = wandb.config.traj
        env_conf["action_pen"] = wandb.config.action
    except:
        print("No sweep is being executed, default coeff for reward are being used")
    
    
    # env = make_vec_env(env_id='f110_gym:f110rl-v0', n_envs=1, env_kwargs=env_conf)
    env = make_vec_env(
        env_id="f110_gym:f110rl-v0", # alternative with (steer, speed) input: `f110_gym:f110rl-v0`
        n_envs=1, 
        env_kwargs=env_conf
    )

    if env_conf["record_video"]:
        env = VecVideoRecorder(env, "recording",
                            record_video_trigger=lambda x: x % int(wandb.config.total_timesteps / 10) == 0,
                            video_length=500)
    env.reset()

    # Choose if you want to finetune a pretrained model
    if continue_training:
        model = SAC.load(
            os.path.join(model_dir, model_name + "/best_model.zip"),
            env=env,
            verbose=1,
            gamma=wandb.config.gamma,
            use_sde=False,
            tensorboard_log="runs/{}".format(run.name),
        )
    else:
        if env_conf['curriculum']:
            model = CurrSAC(
                wandb.config.policy_type,
                env,
                verbose=1,
                tensorboard_log="runs/{}".format(run.name),
                batch_size=64,
                device="auto",
                train_freq=1,
                )
        else:
            model = SAC(
                wandb.config.policy_type,
                env,
                verbose=1,
                tensorboard_log="runs/{}".format(run.name),
                batch_size=64,
                device="auto",
                train_freq=1,
                )

    # CALLBACKS
    wandb_callback = WandbCallback(
        gradient_save_freq=100,
        model_save_freq=int(wandb.config.total_timesteps / 10),
        model_save_path=os.path.join(model_dir, model_name + "/wandb_models/"),
    )

    eval_env_conf = env_conf
    eval_env_conf["ep_len"] = 20000  # evaluation is not "technically" limited in length
    eval_env_conf["random_init"] = False # all evaluation start from the same point
    eval_env_conf["deterministic"] = True
    eval_env = make_vec_env(
        env_id="f110_gym:f110rl-v0", n_envs=1, env_kwargs=eval_env_conf
    )
    vid_rec_trigger = lambda x: x % int(wandb.config.total_timesteps / 10) == 0
    if env_conf["record_video"]:
        eval_env = VecVideoRecorder(
            eval_env,
            "recording",
            record_video_trigger = vid_rec_trigger,
            video_length = 500
            )
    best_model_callback = EvalCallback(
        eval_env,
        best_model_save_path=os.path.join(model_dir, model_name + "/"),
        log_path=os.path.join(model_dir, model_name + "/logs/"),
        eval_freq=wandb.config.total_timesteps / (10),
        deterministic=True,
        render=False,
        n_eval_episodes=1,
    )
    # best_model_callback = CurriculumVelCallback(
    #     eval_env,
    #     best_model_save_path=os.path.join(model_dir, model_name + "/"),
    #     log_path=os.path.join(model_dir, model_name + "/logs/"),
    #     eval_freq=wandb.config.total_timesteps / (10),
    #     deterministic=True,
    #     render=False,
    # )
    metrics_callback = MetricEvalCallback(
        eval_env,
        eval_freq=wandb.config.total_timesteps / (10),
        wandb_run = run,
        )
    curr_callback = CurrCallback(
        wandb_run=run,
        eval_freq=wandb.config.total_timesteps / (100)
        )


    # LEARNING
    model.learn(
        total_timesteps=wandb.config.total_timesteps,
        eval_log_path=os.path.join(dir_path, "train_logs"),
        callback=[
            wandb_callback, 
            metrics_callback,
            best_model_callback,
            curr_callback
            ],
    )

    run.finish()

def init_confs(
    exp_conf,
    ep_len: int = 500,
    params_noise: bool = True,
    ang_deg: int = 30,
    use_trajectory: bool = True,
    max_vel: int = 20,
    record_video: bool = True
):

    with open(os.path.join(configs_dir, "{}.yaml".format(exp_conf["map"]))) as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
        conf = Namespace(**conf_dict)

    env_conf = {
        "map": os.path.join(configs_dir, "{}".format(exp_conf["map"])),
        "map_ext": conf.map_ext,
        "random_init": True,
        "sx": conf.sx,
        "sy": conf.sy,
        "stheta": conf.stheta,
        "num_agents": 1,
        "ep_len": ep_len,  # this sets the maximum episode length, it is ~1.5 times the best time for a lap, so it changes from track to track
        "obs_type": "Frenet", # "original", "Frenet", "Frenet_trajectory",
        "params_noise": params_noise,
        "var_mu": (0.075 / 2) ** 2,  # if all are set to 0 no noise is applied
        "var_Csf": 0,
        "var_Csr": 0,
        "redraw_upon_reset": True,
        "angle_limit": ang_deg * np.pi / 180,  # 30 deg
        "use_trajectory": use_trajectory,
        "max_vel": max_vel,
        "record_video": record_video,
        "curriculum": False, # no curriculum velocity for now
    }
    for k, v in sb3_ppo_config.items():
        env_conf[k] = v

    return env_conf


def init_parser():
    parser = argparse.ArgumentParser(description="Runs the RL train")
    parser.add_argument(
        "-l",
        "--ep_len",
        type=int,
        default=500,
        help="Maximum episode length",
    )
    parser.add_argument(
        "-d",
        "--ang_deg",
        type=int,
        default=30,
        help="Limit slip angle",
    )
    parser.add_argument(
        "-v",
        "--max_vel",
        type=int,
        default=20,
        help="Maximum velocity",
    )
    parser.add_argument(
        "--record_vid",
        dest="record",
        action="store_true",
        help="Activate video recording",
    )
    parser.add_argument(
        "--no-record_vid",
        dest="record",
        action="store_false",
        help="Deactivate video recording",
    )
    parser.set_defaults(record=True)
    parser.add_argument(
        "--p_noise",
        dest="p_noise",
        action="store_true",
        help="Activate noise on dynamics parameters",
    )
    parser.add_argument(
        "--no-p_noise",
        dest="p_noise",
        action="store_false",
        help="Deactivate noise on dynamics parameters",
    )
    parser.set_defaults(p_noise=True)
    parser.add_argument(
        "--use_traj",
        dest="use_traj",
        action="store_true",
        help="Use the trajectory generated by MPC.",
    )
    parser.add_argument(
        "--no-use_traj",
        dest="use_traj",
        action="store_false",
        help="Use the center-line trajectory.",
    )
    parser.set_defaults(use_traj=True)
    
    return parser


if __name__ == "__main__":

    run_mode = "train"
    exp_conf = {
        "mode": "Frenet_trajectory",
        "map": "f",
        "arch": "SAC",
        "conf": sb3_ppo_config,
    }
    logging.basicConfig(
        filename=os.path.join(dir_path, "logs/last_run.log"), level=logging.WARNING
    )

    parser = init_parser()

    args = parser.parse_args()
    env_conf = init_confs(
        exp_conf,
        ep_len=args.ep_len,
        params_noise=args.p_noise,
        ang_deg=args.ang_deg,
        use_trajectory=args.use_traj,
        max_vel=args.max_vel,
        record_video=args.record,
    )

    logging.info("Starting a {}\n".format(run_mode))
    if run_mode == "train":
        train(exp_conf=exp_conf, env_conf=env_conf, continue_training=False)
    logging.info("{} finished.\n".format(run_mode))
