# The PBL F1TENTH Gym environment

This is the repository of the PBL F1TENTH Gym environment.

The original project is still under heavy development, and this project basically branched from it, so do not expect the code to be coherent overall.

You can find the [documentation](https://f1tenth-gym.readthedocs.io/en/latest/) of the original F1TENTH environment here.

## Quickstart
(basically the same as for the original environment)

You can install the environment by running:

```bash
$ git clone <THIS REPO>
$ cd PBL_f1tenth_gym
$ pip3 install -r requirements.txt
$ pip3 install gym/
$ pip3 install splinify_package/
```

## Some notes

### Normalisation library 

It should be possible to use the normalisation module as standalone, by importing
```
from f1tenth_gym.envs import normalisation
```

and then call `normalisation.normalise_trajectory(...)` etc.

### Observation structure 

In the `f110rl-v0` environment, the observation is a 47-d vector consisting of the following elements:
  - 15 points of the trajectory fed to the agent, ordered like so: [x0, y0, x1, y1, ...]
  - the frenet observation, with the following elements: [deviation, rel_heading, longitudinal_vel, later_vel, yaw_rate]
  - the lidar scan, downsampled to only 10 points. The ten points are just taken at equidistant position along the 1080 scan array.
  - the previous input, in the order [steer_v, acceleration]

Note that:
  - when uing the policy network outside the environment, e.g. in ROS, the output must be denormalised (with the appropriate function present in `normalise.py`)
  - to use the acceleration as an input in ROS, `jerk` must be set to 512 
  - steering velocity must be transformed to steering in ROS, e.g. by taking the previous steering command and adding `steering_vel*period_of_controller`


### Some `gym.make` arguments

#### Change observation type
The environment has now a very similar observation space as that in "Learning from Simulation, Racing in Reality" from Chisari et al..
Namely, what they call {p, n, mu, v_x, v_y, omega} in the paper, in our environment becomes, respectively, {param, deviation, rel_heading, longitudinal_vel, later_vel, yaw_rate}.
To activate this action space, one needs to pass the following argument when making the environment (with `gym.make` or else): `obs_type='Frenet`.

#### Various tips and tricks 
The environment has also an episode maximum length, i.e. by setting `ep_len` to, say, 3000 the episode will be stopped if it reaches the 3000-th timestep. 

The starting position can also be randomised by setting `random_init=True`


#### Noise on the tire parameters
Noise on the tire parameters can be added. The parameters to set are: 
  - `params_noise`: a boolean, to choose whether or not apply noise on the tire params
  - `var_mu`: the variance of the noise acting on the friction of the tires
  - `var_Csf`: the variance of the noise acting on the coefficient of stiffness of the front tires
  - `var_Csr`: the variance of the noise acting on the coefficient of stiffness of the rear tires
  - `redraw_upon_reset`: whether to redraw the coefficients upon reset of the environment

According to CommonRoad (the source of the dynamics models) friction is the most important parameter to model weather changes
  
## Known issues
(from the original environment)
- On MacOS Big Sur and above, when rendering is turned on, you might encounter the error:
```
ImportError: Can't find framework /System/Library/Frameworks/OpenGL.framework.
```
You can fix the error by installing a newer version of pyglet:
```bash
$ pip3 install pyglet==1.5.11
```
And you might see an error similar to
```
gym 0.17.3 requires pyglet<=1.5.0,>=1.4.0, but you'll have pyglet 1.5.11 which is incompatible.
```
which could be ignored. The environment should still work without error.

## Citing
If you find this Gym environment useful, please consider citing:

```
@inproceedings{okelly2020f1tenth,
  title={F1TENTH: An Open-source Evaluation Environment for Continuous Control and Reinforcement Learning},
  author={O’Kelly, Matthew and Zheng, Hongrui and Karthik, Dhruv and Mangharam, Rahul},
  booktitle={NeurIPS 2019 Competition and Demonstration Track},
  pages={77--89},
  year={2020},
  organization={PMLR}
}
```
