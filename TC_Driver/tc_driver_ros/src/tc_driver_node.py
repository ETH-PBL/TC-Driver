#!/usr/bin/env python3
import os, yaml, pathlib
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from stable_baselines3 import SAC
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped

from f110_msgs.msg import WpntArray
from f110_gym.envs import normalisation
from base_driver import BaseDriver

class TCDriver(BaseDriver):
    def __init__(self, use_lidar=True) -> None:
        rospy.init_node('tc_driver_node', anonymous=True)

        self.glb_markers = None
        self.glb_wpnts = None
        self.track_bounds = None

        rospy.Subscriber('/global_waypoints', WpntArray, self.glb_wpnts_cb)
        rospy.Subscriber('/global_waypoints/markers', MarkerArray, self.glb_markers_cb)
        rospy.Subscriber('/trackbounds/markers', MarkerArray, self.bounds_cb)
        if use_lidar:
            rospy.Subscriber('/scan', LaserScan, self.lidar_cb)
        rospy.Subscriber('/car_state/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/car_state/pose', PoseStamped, self.pose_cb)

        publish_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_1'
        self.drive_pub = rospy.Publisher(publish_topic, AckermannDriveStamped, queue_size=10)

        # paths
        dir_path = pathlib.Path(__file__).resolve().parents[2]
        self.configs_dir = os.path.join(dir_path, "cfg")
        self.model_dir = os.path.join(dir_path, "models/jfr/slow_car/e2e")

        with open(self.configs_dir + "/NUC3_params.yaml") as file:
            self.car_params = yaml.safe_load(file)

        # RL inits
        self.use_lidar = use_lidar
        self.observation, self.obs_dim = self._init_obs()
        self.model = self.load_model()

        self.cur_steer = 0
        self.prev_action_norm = np.zeros(2)
        self.filtered_scan = None

        # track spline stuff
        self.coords = np.empty((0,2))
        self.left_w = np.empty((0,1))
        self.right_w = np.empty((0,1))
        self.params = np.empty((0,1))
        self.track_length = None
        self.track = None
        self.theta = None # current s of the car

        self.steers = np.zeros((2,1))

        self.frequency = 100

    def load_model(self):
        model_name = 'model_f_SAC_Frenet_trajectory_noise_True/wandb_models/model'
        model_path = os.path.join(self.model_dir, model_name)
        model = SAC.load(model_path, device='cpu')
        return model

    def _init_obs(self):
        # we have : the spatial trajectory (30 x 2) and the Frenet frame (6)
        self._traj_len = 20
        if self.use_lidar:
            obs_dim = self._traj_len*2 + 11 + 5 + 2
        else:
            obs_dim = self._traj_len*2 + 5 + 2 # 10 is the scans, 2 is the previous input normalised TODO improve these numbers i.e. no hard writing
        observation = np.zeros(obs_dim)

        return observation, obs_dim

    def drive(self):
        #Wait until we get the waypoints
        rospy.loginfo('Waiting for global waypoints...')
        rospy.wait_for_message('/global_waypoints', WpntArray)
        rospy.loginfo('Global waypoints obtained!')
        

        rate = rospy.Rate(self.frequency)
        while not rospy.is_shutdown():
            # update s 
            coord = np.array([self.pose.pose.position.x, self.pose.pose.position.y]).reshape(2)
            self.theta = self.track.find_theta(coord, self.theta)

            #RL inference happening
            observation = self.generate_observation()
            action, _states = self.model.predict(observation, deterministic=True)

            steer, vel = normalisation.denorm_action(action=action, params=self.car_params)
            # filter the steering by low passing slightly
            self.steers[1:, :] = self.steers[:-1, :]
            self.steers[0, :] = steer

            vel = np.clip(vel, 0, 3)
            self._pub_drive(steer=np.mean(self.steers, axis=0), speed=vel)
            self.prev_action_norm = action
            rate.sleep()

    def generate_observation(self):
        # trajectory fetch and normalise
        rot_matr = np.array([[np.cos(self.yaw), np.sin(self.yaw)],[-np.sin(self.yaw), np.cos(self.yaw)]]) # rotates the points so that x axis is aligned with car
        trajectory = [rot_matr@(np.array(self.track.trajectory.get_coordinate(th))-self.position) for th in np.linspace(self.theta, self.theta+self._traj_len/10, num=self._traj_len)]
        trajectory_norm = normalisation.normalise_trajectory(trajectory, self._traj_len)

        # frenet fetch and normalise
        obs = {}
        if self.use_lidar:
            obs["scans"] = self.filtered_scan
        obs["deviation"] = np.linalg.norm(self.position-self.track.trajectory.get_coordinate(self.theta))
        obs["rel_heading"] = self.get_rel_head()
        obs["longitudinal_vel"] = self.ekf_odom.twist.twist.linear.x
        obs["later_vel"] = self.ekf_odom.twist.twist.linear.y
        obs["yaw_rate"] = self.ekf_odom.twist.twist.angular.z
        obs_norm = normalisation.normalise_observation(obs, self.car_params, with_lidar=self.use_lidar)
        if self.use_lidar:
            keys = ['scans', 'deviation', 'rel_heading', 'longitudinal_vel', 'later_vel', 'yaw_rate']
        else:
            keys = ['deviation', 'rel_heading', 'longitudinal_vel', 'later_vel', 'yaw_rate']
        obs_norm_flat = np.concatenate([obs_norm[key] for key in keys], axis=None).astype(np.float32)
        
        self.observation = np.concatenate((trajectory_norm, obs_norm_flat, self.prev_action_norm), axis = None) # obtain trajectory

        return self.observation

    def get_rel_head(self):
        """
        Obtain heading of the car relative to the track, handling wrapping
        """

        angle = self.track.get_angle(self.theta)
        if angle < 0:
            print("Really? angle should always be > 0")
            angle+=2*np.pi
        if not np.isnan(self.yaw):
            car_angle = self.yaw%(2*np.pi)
        else:
            car_angle = 0

        rel_head = car_angle - angle
        
        if rel_head>=np.pi:
            rel_head -= 2*np.pi
        elif rel_head<= -np.pi:
            rel_head += 2*np.pi

        return rel_head


if __name__ == '__main__':
    tc_driver = TCDriver(use_lidar=True)
    tc_driver.drive()
