#!/usr/bin/env python3
import os
import yaml
import rospy
import pathlib
import numpy as np
from nav_msgs.msg import Odometry
from stable_baselines3 import SAC
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped

from base_driver import BaseDriver
from f110_msgs.msg import WpntArray
from f110_gym.envs import normalisation


class E2EDriver(BaseDriver):
    def __init__(self) -> None:
        """
        General structure for an RL ROS node
        """
        rospy.init_node('e2e_driver_node', anonymous=True)

        # subscribe to relevant topic
        rospy.Subscriber('/scan', LaserScan, self.lidar_cb)
        rospy.Subscriber('/global_waypoints', WpntArray, self.glb_wpnts_cb)
        rospy.Subscriber('/car_state/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/car_state/pose', PoseStamped, self.pose_cb)
        
        # instantiate publishers to relevant topic here
        publish_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_1'
        self.drive_pub = rospy.Publisher(publish_topic, AckermannDriveStamped, queue_size=10)

        # others
        self.pose = None
        self.theta = None
        self.track = None
        self.track_length = None
        self.coords = np.empty((0,2))
        self.left_w = np.empty((0,1))
        self.right_w = np.empty((0,1))
        self.params = np.empty((0,1))
        self.frequency = 50
        dir_path = pathlib.Path(__file__).resolve().parents[2]
        self.model_dir = os.path.join(dir_path, "models/jfr/e2e")
        self.configs_dir = os.path.join(dir_path, "cfg")

        with open(self.configs_dir + "/NUC3_params.yaml") as file:
            self.car_params = yaml.safe_load(file)

        # RL inits
        self.observation, self.obs_dim = self._init_obs()
        self.model = self.load_model()

        self.steers = np.zeros((2, 1))
    
    def load_model(self):
        model_name = 'model_f_SAC_Frenet_noise_True/model_last'
        model_path = os.path.join(self.model_dir, model_name)
        model = SAC.load(model_path, device='cpu')
        return model

    def _init_obs(self):
        # we have : the spatial trajectory (30 x 2) and the Frenet frame (6)
        self.lidar_len = 11
        self.frenet_len = 5
        obs_dim = self.lidar_len + self.frenet_len #  lidar + frenet frame
        observation = np.zeros(obs_dim)

        return observation, obs_dim
    
    def generate_observation(self):
        # frenet fetch and normalise
        obs = {}
        obs["scans"] = self.filtered_scan
        obs["deviation"] = np.linalg.norm(self.position-self.track.trajectory.get_coordinate(self.theta))
        obs["rel_heading"] = self.get_rel_head()
        obs["longitudinal_vel"] = self.ekf_odom.twist.twist.linear.x
        obs["later_vel"] = self.ekf_odom.twist.twist.linear.y
        obs["yaw_rate"] = self.ekf_odom.twist.twist.angular.z
        obs_norm = normalisation.normalise_observation(obs, self.car_params, with_lidar=True)
        keys = ['scans', 'deviation', 'rel_heading', 'longitudinal_vel', 'later_vel', 'yaw_rate']
        self.observation = np.concatenate([obs_norm[key] for key in keys], axis=None).astype(np.float32)
        
        return self.observation

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

            self.steers[1:,:] = self.steers[:-1, :]
            self.steers[0, :] = steer

            vel = np.clip(vel, 0, 3)
            self._pub_drive(steer=np.mean(self.steers, axis=0), speed=vel)
            self.prev_action_norm = action
            rate.sleep()

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
    e2e_driver = E2EDriver()
    e2e_driver.drive()
