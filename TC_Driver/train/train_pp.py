import os, time, yaml, logging, pathlib
from argparse import Namespace

import copy
import wandb
import numpy as np
from xvfbwrapper import Xvfb
from wandb.integration.sb3 import WandbCallback
from stable_baselines3 import SAC
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback

from utils import MetricEvalCallback
import cl_parser


# paths
dir_path = pathlib.Path(__file__).resolve().parents[1]
configs_dir = os.path.join(dir_path, "cfg")
model_dir = os.path.join(dir_path, "models/pprl/slow")


def train(env_conf, continue_training=False):
    model_name = "model_pprl_{}_SAC_{}_noise_{}".format(
        env_conf["map_name"], env_conf["obs_type"], str(env_conf["params_noise"])
    )

    project = "PPRL"
    
    # Wandb init for logging
    run = wandb.init(
        name="{}_{}_{}_noise_{}_{}".format(
            env_conf["obs_type"],
            env_conf["arch"],
            env_conf["map_name"],
            str(env_conf["params_noise"]),
            int(time.time()),
        ),
        project=project,
        entity="forzapbl",
        config=env_conf,
        sync_tensorboard=True,  # auto-upload sb3's tensorboard metrics
        monitor_gym=True,  # auto-upload the videos of agents playing the game
        save_code=True,
        mode="run", # can be `disabled`, `offline`, `run`
    )  
    wandb.run.log_code(".")
    try:
        env_conf["traj_coeff"] = wandb.config.traj
        env_conf["action_pen"] = wandb.config.action
    except:
        print("No sweep is being executed, default coeff for reward are being used")
    
    
    env = make_vec_env(
        env_id=env_conf['env_id'],
        n_envs=1, 
        env_kwargs=env_conf
    )

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
            reg_matrix = env_conf['output_reg'],
            device="cpu"
        )
    else:
        model = SAC(
            wandb.config.policy_type,
            env,
            verbose=1,
            tensorboard_log="runs/{}".format(run.name),
            batch_size=64,
            device="cpu",
            train_freq=1,
            # reg_matrix = env_conf['output_reg']
            )

    # CALLBACKS
    wandb_callback = WandbCallback(
        gradient_save_freq=100,
        model_save_freq=int(wandb.config.total_timesteps / 10),
        model_save_path=os.path.join(model_dir, model_name + "/wandb_models/"),
    )

    eval_env_conf = copy.deepcopy(env_conf)
    eval_env_conf["ep_len"] = 10000  # evaluation is not "technically" limited in length
    eval_env_conf["random_init"] = False # all evaluation start from the same point
    eval_env_conf["deterministic"] = True
    eval_env = make_vec_env(
        env_id=env_conf["env_id"], n_envs=1, env_kwargs=eval_env_conf
    )
    eval_env.training = False
    vid_rec_trigger = lambda x: x % int(wandb.config.total_timesteps / 10) == 0
    best_model_callback = EvalCallback(
        eval_env,
        best_model_save_path=os.path.join(model_dir, model_name + "/"),
        log_path=os.path.join(model_dir, model_name + "/logs/"),
        eval_freq=wandb.config.total_timesteps / (10),
        deterministic=True,
        render=False,
        n_eval_episodes=1,
    )
    metrics_callback = MetricEvalCallback(
        eval_env,
        eval_freq=wandb.config.total_timesteps / (10),
        wandb_run = run,
        )
    


    # LEARNING
    model.learn(
        total_timesteps=wandb.config.total_timesteps,
        eval_log_path=os.path.join(dir_path, "train_logs"),
        callback=[
            wandb_callback, 
            metrics_callback,
            best_model_callback
        ],
    )

    # SAVING the final model
    model.save(os.path.join(model_dir, model_name + "/model_last"))

    run.finish()

def init_confs(
    mode,
    map,
    arch,
    ep_len: int = 500,
    params_noise: bool = True,
    ang_deg: int = 30,
    use_trajectory: bool = True,
    max_vel: int = 20,
    display_video: bool = True
):

    with open(os.path.join(configs_dir, "{}.yaml".format(map))) as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
        conf = Namespace(**conf_dict)

    with open(os.path.join(configs_dir, "car_params.yaml".format(map))) as file:
        params = yaml.safe_load(file)

    env_conf = {
        "mode": mode,
        "arch": arch,
        "map_name": map,
        "map": os.path.join(configs_dir, "{}".format(map)),
        "map_ext": conf.map_ext,
        "random_init": True,
        "sx": conf.sx,
        "sy": conf.sy,
        "stheta": conf.stheta,
        "num_agents": 1,
        "ep_len": ep_len,  # this sets the maximum episode length, it is ~1.5 times the best time for a lap, so it changes from track to track
        "obs_type": mode,
        "params_noise": params_noise,
        "var_mu": (0.075 / 2) ** 2,  # if all are set to 0 no noise is applied
        "var_Csf": 0,
        "var_Csr": 0,
        "redraw_upon_reset": True,
        "angle_limit": ang_deg * np.pi / 180,  # 30 deg
        "use_trajectory": use_trajectory,
        "max_vel": max_vel,
        "display_video": display_video, 
        "curriculum": False, # no curriculum velocity for now
        "policy_type": "MlpPolicy",
        "total_timesteps": 1e6,
        "gamma": 0.99,
        "env_id":"f110_gym:f110pp-v0",
        "output_reg":np.diag([0.0]),
        "params": params,
        "use_speed": True,
        "action_noise": NormalActionNoise(0, 0.05)
        # "ent_coef": 0.69
    }
    return env_conf

if __name__ == "__main__":

    parser = cl_parser.init_parser()

    args = parser.parse_args()
    env_conf = init_confs(
        mode="trajectory", # "trajectory" is the only available
        map="f",
        arch="SAC",
        ep_len=args.ep_len,
        params_noise=args.p_noise,
        ang_deg=args.ang_deg,
        use_trajectory=args.use_traj,
        max_vel=args.max_vel,
        display_video=args.display,
    )

    logging.basicConfig(
        filename=os.path.join(dir_path, f"logs/last_run_{env_conf['mode']}.log"),
        level=logging.INFO
    )

    logging.info("Starting a training run\n")
    # we need to use a virtual screen
    if args.display:
        train(env_conf=env_conf, continue_training=False)
    else:
        with Xvfb() as xvfb:
            train(env_conf=env_conf, continue_training=False)
    logging.info("Training finished.\n")
