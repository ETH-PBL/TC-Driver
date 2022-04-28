# TC-Driver: Trajectory Conditioned Driving for Robust Autonomous Racing - A Reinforcement Learning Approach

A model-free RL approach to tackle model missmatch and enhance track generalisation in autonomous racing. Instead of end-to-end RL architectures (control output learned directly from sensory input), we can leverage the reliability of traditional planning methods to interact with a low level RL agent that has learned how to drive/race under varying model parameters. This allows for robustness against model mismatch and greater generalisation towards unseen tracks.

This repository contains the code to reproduce the proposed TC-Driver, as well as the benchmark MPCC and end-to-end architecture in the [F1TENTH Gym environment](https://github.com/f1tenth/f1tenth_gym).  

### Setup 
First install the requirements contained in the `requirements.txt` with something like this
```
    pip3 install -r requirements.txt
```
If you wish, do it in a python environment

Then navigate to the folder `./pbl_f110_gym/gym/` and install the modified gym environment with 
```
pip install .
```

Then, install the splinify package by navigating to the folder `./pbl_f110_gym/splinify_package`
and install the package with
```
pip install .
```

### Running something
The main script to test the beavior of the MPCC controller is car_tester.py in /f1tenth_IROS2021. No arguments are curently available, just call 
```
python ./f1tenth_IROS2021/car_tester.py
```
(Assuming you are in the root folder of the repository)
