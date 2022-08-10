# We are still working on the official deployment of this repo, sorry for the delay!!!



# TC-Driver: Trajectory Conditioned Driving for Robust Autonomous Racing - A Reinforcement Learning Approach

A model-free RL approach to tackle model missmatch and enhance track generalisation in autonomous racing. Instead of end-to-end RL architectures (control output learned directly from sensory input), we can leverage the reliability of traditional planning methods to interact with a low level RL agent that has learned how to drive/race under varying model parameters. This allows for robustness against model mismatch and greater generalisation towards unseen tracks and demonstrates zero-shot Sim2Real capabilities on a physical F1TENTH race car.

This repository contains the code to reproduce the proposed TC-Driver, as well as the benchmark MPCC and End2End architecture in the [F1TENTH Gym environment](https://github.com/f1tenth/f1tenth_gym).  

# Simulation Results







# Model Free Zero-Shot Sim2Real Capabilities
The proposed TC-Driver RL agent is trained in simulation only and can be deployed on a physical car, on an unseen track and complete laps with similar crash-ratio as observed in simulation. In the image below, you can see the physical 1:10 scaled F1TENTH car, along with an example track on which it was deployed. 

<img src="/misc/imgs/titlepage_car.png" width="500">

Here you can see the RVIZ visualisation of TC-Driver and the End2End learned architectures. TC-Driver can perform the 10 laps with a single crash, while the End2End architecture fails to complete a single lap without crash. Therefore TC-Driver demonstrates a 10% crash ratio on this track, while End2End has a 100% crash ratio. The rosbag recordings are uploaded [here](/TC_Driver/plotting/).

| TC-Driver | End2End |
| ------ | ------ |
| ![TC](/misc/imgs/tc_rviz.gif) | ![E2E](/misc/imgs/e2e_rviz.gif) |

![Track](/misc/imgs/real_traj.png)

_Note: Different track recordings following._

Lastly here is a gif of the TC-Driver completing it's track :)
![TC_video](/misc/imgs/tc_video.gif)

