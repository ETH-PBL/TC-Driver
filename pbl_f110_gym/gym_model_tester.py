import pathlib, os, time, yaml
import gym
from argparse import Namespace

from stable_baselines3 import PPO


dir_path = pathlib.Path(__file__).parent.resolve()
model_dir = 'logs/best_model'
configs_dir = os.path.join(dir_path, 'configs')


def testo(map_name):
    model = PPO.load(os.path.join(dir_path, model_dir))
    with open(os.path.join(configs_dir, '{}.yaml'.format(map_name))) as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
    conf = Namespace(**conf_dict)
    env = gym.make('f110_gym:f110rl-v0',
                   map=conf.map_path,
                   map_ext=conf.map_ext,
                   sx=conf.sx,
                   sy=conf.sy,
                   stheta=conf.stheta,
                   num_agents=1)
    done = False
    laptime = 0.0
    start = time.time()

    obs = env.reset()
    while not done:
        action, _states = model.predict(obs, deterministic=True)
        obs, step_reward, done, info = env.step(action)
        print('Reward: ', step_reward)
        print('Action: ', action)
        laptime += 0.01
        env.render(mode='human_fast')
        #time.sleep(1)
    print('Sim elapsed time:', laptime, 'Real elapsed time:', time.time() - start)


if __name__ == '__main__':
    map_name = 'circle'
    testo(map_name=map_name)
    