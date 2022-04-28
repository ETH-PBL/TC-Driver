import random
import numpy as np

PENALTY = -1e3

def get_reward_time(sim, ego_idx, next_waypoint, last_visited_times, track, current_time, times) -> int:
    """
    Obtains the reward, based on waypoints situated at theta == int(theta).
    Rewards the improvement on the last track time.
    
    Returns:
        reward: negative delta from last lap time to best lap time at the current waypoint
    """

    reward = 0
    car = sim.agents[ego_idx]

    car_position = car.state[:2]
    idx = next_waypoint - 1 
    #print(next_waypoint)

    if track.is_coord_behind(car_position, idx):
        # going backwards
        # therefore go back with the waypoint and reset the timer 
        next_waypoint -= 1
        last_visited_times[next_waypoint] = -1

        if next_waypoint < 0:
            if next_waypoint != -1:
                print("weird shit while crossing the starting line backwards!")
            else:
                next_waypoint = int(np.floor(track.track_length))
        # give penalty
        print("feels like we only go backwards")
        reward = PENALTY

    elif not track.is_coord_behind(car_position, next_waypoint):
        
        if not last_visited_times[next_waypoint] == -1:
            last_lap_time =  (current_time - last_visited_times[next_waypoint])
            if random.random() > 0.99:
                print(last_lap_time)
            reward =  - (last_lap_time - times[next_waypoint]) + 1 # there is a 1 second slack

            # scaling of time reward
            if reward >= 0:
                reward = 10 ** (3 + 0.58*np.log(reward + 1)) # this was designed so that : 0-1 s improvements -> ~1e3 rew / ~5s impr. -> ~ 1e4 rew / 20s impr. -> 1e5 rew. 
            else:
                reward *= 1e3 # make slower lap times a bit worse 

            #if last_lap_time <= times[next_waypoint]:
                #times[next_waypoint] = last_lap_time
        else:
            reward = 100
        last_visited_times[next_waypoint] = current_time
        next_waypoint += 1
        if next_waypoint >= track.track_length + 1:
            raise ValueError("Weird, maybe the car is too fast?")
        if next_waypoint >= track.track_length:
            next_waypoint = 0

        if not track.is_coord_behind(car_position, idx+2):
            raise ValueError("Other weird shit is happening!") # TODO remove this if it works correctly

    #print(self.next_waypoint)
    return reward, next_waypoint, last_visited_times
    
def get_reward_vel(sim, ego_idx):
    """
    Returns the current velocity of the car as a reward 
    """
    return sim.agents[ego_idx].state[3]

def get_reward_adv(theta, prev_theta):
    """
    Returns the advancement of last timestep calculated along the central line
    """
    return (theta - prev_theta) 

def get_pseudocurr_reward(time_rew, speed_rew, adv_rew, sim, ego_idx):
    """
    Inspired to curriculum RL, it should first train the agent to do something simple (go forward quick) and then switch to 
    something more involved (make better lap times)

    ideally advancement is 50% less important than speed at the beginning, but both are scaled around 0-100 rewards
    when laps are done then lap time becomes crucial, being scaled around 1e3 - 1e5
    """

    slip = abs(sim.agents[ego_idx].state[-1])

    return 600*adv_rew + (120)*speed_rew + 100*time_rew - 690*slip

def get_reward_safety(sim, ego_idx, track, theta):
    """
    returns a negative reward (penalty), for going too distant for the center of the track
    it is approximately: 
        - lesser than one when distance to center is < 80% track width
        - much greater than one, elsewhere
    """
    perc=0.8 # percentual can be changed here

    dist = np.linalg.norm(np.array(sim.agents[ego_idx].state[:2]) - np.array(track.get_coordinate(theta)))
    safe_width = perc*np.linalg.norm(np.array(track.get_coordinate(theta, line='int')) - np.array(track.get_coordinate(theta)))
    return -(dist/safe_width)**10

def get_pseudoscaramuzza_reward():
    """
    Reward inspired by "Autonomous Drone Racing with Deep Reinforcement Learning" by Song et al.
    """
    part_adv = 100*get_reward_adv()
    part_vel = get_reward_vel()
    part_saf = get_reward_safety()
    #print(part_adv, part_vel, part_saf)
    return part_adv + part_vel + part_saf
