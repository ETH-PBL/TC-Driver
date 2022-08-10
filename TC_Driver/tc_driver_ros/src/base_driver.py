#!/usr/bin/env python3
import os, yaml, pathlib
from abc import ABC, abstractclassmethod, abstractmethod
from argparse import Namespace

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from stable_baselines3 import SAC
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from splinify.splinify import SplineTrackNew
from scipy.spatial.transform import Rotation
from visualization_msgs.msg import MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped
from stable_baselines3.common.env_util import make_vec_env

from f110_msgs.msg import WpntArray
from f110_gym.envs import normalisation


class BaseDriver(ABC):
    @abstractmethod
    def __init__(self) -> None:
        """
        General structure for an RL ROS node
        """

        # init the node
        # rospy.init_node(<node name here>, anonymous=True)

        # subscribe to relevant topic
        # rospy.Subscriber(<relevant topic name here>, <datatype here>, <callback to handle subscriptions here>)
        
        # instantiate publishers to relevant topic here
        # self.generic_publisher = rospy.Publisher(<topic name>, <datatype>, queue_size=10)

        # others
        # self.foo = None

        pass
    
    @abstractmethod
    def load_model(self):
        pass

    @abstractmethod
    def _init_obs(self):
        pass

    @abstractmethod
    def drive(self):
        pass

    @abstractmethod
    def generate_observation(self):
        pass

    def odom_cb(self, odom:Odometry):
        self.ekf_odom = odom

    def pose_cb(self, pose: PoseStamped):
        self.pose = pose
        self.position = np.array([pose.pose.position.x, pose.pose.position.y]).reshape(2,1)
        quat = Rotation.from_quat([
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
            ])
        self.yaw = quat.as_euler('zyx')[0] # in radians

    def lidar_cb(self, scans:LaserScan):
        ranges = scans.ranges
        self.filtered_scan = [ranges[180+4*18*i] for i in range(11)]

    def glb_wpnts_cb(self, data:WpntArray):
        self.glb_wpnts = data

        if self.track is None:
            # parsing the waypoint array
            for wpnt in self.glb_wpnts.wpnts:
                self.coords = np.concatenate(( self.coords, np.array(((wpnt.x_m, wpnt.y_m))).reshape(-1, 2) ), axis=0)
                self.left_w = np.concatenate(( self.left_w, np.array((wpnt.d_left)).reshape(-1, 1) ), axis=0)
                self.right_w = np.concatenate(( self.right_w, np.array((wpnt.d_right)).reshape(-1, 1) ), axis=0)
                self.params = np.concatenate(( self.params, np.array((wpnt.s_m)).reshape(-1, 1) ), axis=0)

            self.track_length = self.params[-1] + np.linalg.norm(self.coords[0, :]-self.coords[-1,:])
            #self.params = np.concatenate(( self.params, np.array((self.track_length)).reshape(-1, 1) ), axis=0)

            self.track = SplineTrackNew(
                    coords_param_direct={
                        "coords": self.coords,
                        "left_widths": self.left_w,
                        "right_widths": self.right_w
                    },
                    params = self.params,
                    track_length=self.track_length
                )
            
        if self.theta is None and self.pose is not None:
            # initialising the s 
            coord = np.array((self.pose.pose.position.x, self.pose.pose.position.y)).reshape(2,1)
            self.theta = self.track.find_theta_slow(coord)

    def glb_markers_cb(self, data):
        self.glb_markers = data

    def bounds_cb(self, data):
        self.track_bounds = data

    def _pub_drive(self, speed, steer):
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.header.frame_id = 'base_link'
        ack_msg.drive.steering_angle = steer
        ack_msg.drive.speed = speed
        self.drive_pub.publish(ack_msg)

    def _pub_drive_acc(self, acc, steer):
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.header.frame_id = 'base_link'
        ack_msg.drive.steering_angle = steer
        ack_msg.drive.acceleration = acc
        ack_msg.drive.jerk = 512
        self.drive_pub.publish(ack_msg)

    