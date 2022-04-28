# TC-Driver: Trajectory Conditioned Driving for Robust Autonomous Racing - A Reinforcement Learning Approach

A model-free RL approach to tackle model missmatch and enhance track generalisation in autonomous racing. Instead of end-to-end RL architectures (control output learned directly from sensory input), we can leverage the reliability of traditional planning methods to interact with a low level RL agent that has learned how to drive/race under varying model parameters. This allows for robustness against model mismatch and greater generalisation towards unseen tracks.

This repository contains the code to reproduce the proposed TC-Driver, as well as the benchmark MPCC and end-to-end architecture in the [F1TENTH Gym environment](https://github.com/f1tenth/f1tenth_gym).  


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
$ pip3 install -e gym/
```
## Some notes

### Some `gym.make` arguments

#### Change observation type
The environment has now a very similar observation space as that in "Learning from Simulation, Racing in Reality" from Chisari et al..
Namely, what they call {p, n, mu, v_x, v_y, omega} in the paper, in our environment becomes, respectively, {param, deviation, rel_heading, longitudinal_vel, later_vel, yaw_rate}.
To activate this action space, one needs to pass the following argument when making the environment (with `gym.make` or else): `obs_type='Frenet`.

#### Training the agents
To re-produce the results the four different agents have to be trained. 
To train an agent one has to run the f110_rl.py script.
Changing the env_conf in init_confs to reflect the type of agent one wants to train allows for all four combinations to be trained. Changing the 'obs_type' from 'Frenet_trajectory' to 'original' allows on to either train the trajectory tracker or end-to-end agent. Changing 'params_noise' from False to True allows the injection of noise into the training.

#### Evaluation the agents
The agents were evaluated using the test function in f110_rl.py. It is called by setting run_mode = 'train' to 'test' instead. For plotting the images there is the plot_runs branch containing a plot_runs.ipynb notebook generating different images. If test is run in this branch the taken trajectory is saved at the end of the test function.

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
