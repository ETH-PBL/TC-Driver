# The F1TENTH - Riders

Sample project for the F1Tenth challenges, most recent being F1Tenth IROS 2021 (https://riders.ai/challenge/47/f1-tenth-iros-2021/). 

## Installation

Clone this repository and install required packages:

```bash
git clone https://gitlab.com/acrome-colab/riders-poc/f1tenth-riders-quickstart --config core.autocrlf=input
cd f1tenth-gym-quickstart
pip install --user -e gym
```

Finally, check if the repo is working properly:

```bash
cd pkg/src
python -m pkg.main
```

## Entering the Competition

* Signup to the competition platform [Riders.ai](http://riders.ai/) (if you haven't already)
* Go to [My Account > Purchases](https://riders.ai/account/purchases)
* Enter the Redeem Code you have been provided (or ask an organizer for one!)
* Once the (free) purchase is successful, go to your Riders.ai Dashboard, now you should be able to see F1Tenth IROS 2021 Challenge there
* Follow that link and click on Join Competition (Note: this creates a Challenge Group, and makes you the admin of that group. If you want multiple people to compete on the same Team, one of you should Join, and then invite the others from Challenge > My Team page).

## Developing your Driver

To develop your driver you can work in the folder [pkg/src/pkg](./pkg/src/pkg).

### Structure of a Driver

Let's take a look at the most basic Driver, which is in the file [drivers.py](./pkg/src/pkg/drivers.py)

```python
class SimpleDriver:    

    def process_lidar(self, ranges):
        speed = 5.0
        steering_angle = 0.0
        return speed, steering_angle
```

A Driver is just a class that has a ```process_lidar``` function which takes in LiDAR data and returns a speed to drive at along with a steering angle.

```ranges```: an array of 1080 distances (ranges) detected by the LiDAR scanner. As the LiDAR scanner takes readings for the full 360&deg;, the angle between each range is 2&pi;/1080 (in radians).

```steering_angle```: an angle in the range [-&pi;/2, &pi;/2], i.e. [-90&deg;, 90&deg;] in radians, with 0&deg; meaning straight ahead.

### Choosing a Driver

Let's look at the [main.py](./pkg/src/pkg/main.py) file. The section shown below is all we need to worry about.

```python
...
# import your drivers here
from pkg.drivers import DisparityExtender

# choose your drivers here (1-4)
drivers = [DisparityExtender()]

# choose your racetrack here (SILVERSTONE, SILVERSTONE_OBS)
RACETRACK = 'SILVERSTONE'
...
```

As shown in the comments above, we can import Drivers and then choose which ones we want to use. Let's import our SimpleDriver and choose it

```python
...
# import your drivers here
from pkg.drivers import DisparityExtender, SimpleDriver

# choose your drivers here (1-4)
drivers = [SimpleDriver()]
...
```

Now if you run the main.py file again, it uses our SimpleDriver

```bash
$ python main.py
```

To see some more complex processing, take a look at the GapFollower Driver which implements the [Follow The Gap Method](https://www.youtube.com/watch?v=7VLYP-z9hTw&ab_channel=Real-TimemLABUPenn)! Notice that it still has a ```process_lidar``` function which takes in LiDAR data and returns a speed and steering angle. That's all we'll ever need.

### Multi-Agent Racing

To practice racing multiple Drivers against eachother, simply choose multiple Drivers! You may choose up to 4 drivers, but in practice the simulator will usually run very slowly if you choose more than 2. You may race the same Driver against itself by choosing it twice. If you try racing GapFollower against itself, you will find that it is not good at multi-agent racing! 

Here's how we would race GapFollower against SimpleDriver:

```python
# import your drivers here
from pkg.drivers import GapFollower, SimpleDriver

# choose your drivers here (1-4)
drivers = [GapFollower(), SimpleDriver()]

# choose your racetrack here (SILVERSTONE, SILVERSTONE_OBS)
RACETRACK = 'SILVERSTONE'
```

### Changing Map

You may choose between using the ordinary Silverstone map or the Silverstone Obstacles map. These are the two maps that will be used in the competition. To switch between them simply change the name of the selected `RACETRACK`

```python
...
# choose your racetrack here (SILVERSTONE, SILVERSTONE_OBS)
RACETRACK = 'SILVERSTONE_OBS'
...
```

### Baseline Solution

The baseline solution for this competition is the DisparityExtender, which is included in the [drivers.py](./pkg/src/pkg/drivers.py) file. This Driver is an implementation of the [Disparity Extender Algorithm](https://www.nathanotterness.com/2019/04/the-disparity-extender-algorithm-and.html) which has proved successful in previous competitions.

This baseline should already pass the obstacle avoidance track as-is but it's not very fast! Speeding it up will introduce new challenges which can be handled with some thinking. Each function in this baseline also has tips on ways it can be improved.

You don't need to use the baseline solution but if you're not sure where to start this is a good place!

## Preparing a Submission

Prerequisites installs for submitting are: [Docker](https://www.docker.com/products/docker-desktop), [jq](https://stedolan.github.io/jq/)

Use the file [pkg/nodes/f1tenth_ros_agent.py](pkg/nodes/f1tenth_ros_agent.py) to choose the driver you are submitting, as shown below, where we choose the DisparityExtender driver:

```python
...
from pkg.drivers import DisparityExtender as Driver
...
```

Create an `.env` file at the root of the project with following contents:

```bash
RACE_MAP_PATH=/catkin_ws/src/f1tenth_gym_ros/maps/SILVERSTONE.yaml
RACE_MAP_IMG_EXT=.png
F1TENTH_AGENT_NAME=a1
F1TENTH_AGENT_IMAGE=a1
RIDERS_CHALLENGE_ID=47
RIDERS_API_HOST=https://api.riders.ai
RIDERS_F1TENTH_HOST=https://f1tenth.riders.ai
```

Then, from the root of the project, build your submission:

```bash
docker-compose build agent
```

## Testing a Submission

The submission platform uses ROS to run the cars. Your car should race almost exactly the same in ROS as it did in the environment used in `Developing your Driver`, but it is a good idea to double-check sometimes by using ROS locally. This section will show you how to test your submission (if you want to) before you upload it.

Note: choose between `SILVERSTONE.yaml` and `SILVERSTONE_OBS.yaml` in the `.env` file shown above to choose which map to test on (this will not have an effect on what map is used when you submit)

Start ROSCore & F1Tenth ROS Bridge:

```bash
docker-compose up --force-recreate roscore-dev bridge-dev
```

Go to http://localhost:6080 , if everything worked properly until now, you should see simulator window. 

Finally, from another terminal, launch the Driver agent:   

```bash
docker-compose up --force-recreate agent-dev
``` 

You should see your agent start driving along the track.

## Submitting

Requirements for running the submission script: 

Move into the [scripts](./scripts) directory and run the submission file:

```bash
cd scripts
sh submit.sh
```

Follow the instructions displayed by the script.

Once it's finished, check the competition page to see how you did! (it may take up to 15 minutes to process your new submission)

### Submission in Windows (With Docker)

If you already have Docker, just run `submit-with-docker.sh` in project root:

```bash
bash scripts/submit-with-docker.sh
```

If you don't have Docker or get any error, follow this instructions.

- Visit https://www.docker.com/products/docker-desktop and install Docker Desktop.
- To be sure, start Docker Desktop from the Windows Start menu. Then, from the Docker menu, select **Settings > General**. If you have installed Docker Desktop successfully, **Use WSL 2 based engine** check box will be checked by default. If not, check and click **Apply & Restart**.
- After that, visit here https://docs.microsoft.com/en-us/windows/wsl/install-win10#step-4---download-the-linux-kernel-update-package
- Perform steps 4-5-6 at the above address
- Then, to check the WSL mode, run :

```bash
wsl.exe -l -v
```
- Here, you can see the linux distro you installed in step 6. You need to set your chosen distro as default, run:

```bash
wsl --set-default <distro name>
```

For example, to set Ubuntu as your default WSL distro, run:

 ```bash
 wsl --set-default ubuntu
```

- Installation is done! Finally, run `submit-with-docker.sh` in project root:

```bash
bash scripts/submit-with-docker.sh
```

### Submission in Windows (with Python)

If previous options don't work, you can try to upload with Python script:

```bash
pip install docker six
python scripts/submit-with-docker.py
``` 
Enter your credentials and this script should start a docker container in backend that completes submission. After submission make sure that you visit Submissions page to validate newly created submission.
In 15 minutes, visit Results page to view results of your agent.

## Known issues (from original repo)

- If you run the `pip install...` command above and then later change your file structure in some way, you may get errors with `gym` such as `module 'gym' has no attribute 'make'`. The solution to this is to re-run the command `pip install --user -e gym/`.

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
  author={O�Kelly, Matthew and Zheng, Hongrui and Karthik, Dhruv and Mangharam, Rahul},
  booktitle={NeurIPS 2019 Competition and Demonstration Track},
  pages={77--89},
  year={2020},
  organization={PMLR}
}
```
