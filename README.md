# TC-Driver: Trajectory Conditioned Driving for Robust Autonomous Racing - A Reinforcement Learning Approach

A model-free RL approach to tackle model missmatch and enhance track generalisation in autonomous racing. Instead of end-to-end RL architectures (control output learned directly from sensory input), we can leverage the reliability of traditional planning methods to interact with a low level RL agent that has learned how to drive/race under varying model parameters. This allows for robustness against model mismatch and greater generalisation towards unseen tracks.

This repository contains the code to reproduce the proposed TC-Driver, as well as the benchmark MPCC and end-to-end architecture in the [F1TENTH Gym environment](https://github.com/f1tenth/f1tenth_gym).  
