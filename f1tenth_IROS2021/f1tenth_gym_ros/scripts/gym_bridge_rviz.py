#!/usr/bin/env python
import rospy
import os
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped

from f1tenth_gym_ros.msg import RaceInfo

from tf2_ros import transform_broadcaster
from tf.transformations import quaternion_from_euler

import numpy as np
import gym


class GymBridge(object):
    def __init__(self):
        # get env vars
        self.race_scenario = os.environ.get('RACE_SCENARIO')
        if self.race_scenario is None:
            print('Race scenario not set! Using single car timed trial as default.')
            self.race_scenario = 0
        else:
            self.race_scenario = int(self.race_scenario)
        self.ego_id = os.environ.get('EGO_ID')
        self.opp_id = os.environ.get('OPP_ID')
        if self.opp_id is None:
            print('Opponent id not set! Using opp_id as default.')
            self.opp_id = 'opp_id'

        # initialize parameters

        # Topic Names
        self.race_info_topic = rospy.get_param('race_info_topic')
        self.ego_scan_topic = '/' + self.ego_id + '/scan'
        self.ego_odom_topic = '/' + self.ego_id + '/odom'
        self.opp_odom_topic = '/' + self.opp_id + '/odom'
        self.ego_drive_topic = '/' + self.ego_id + '/drive'
        if self.race_scenario:
            # 2 car grand prix, need extra topics
            self.opp_scan_topic = '/' + self.opp_id + '/scan'
            self.ego_opp_odom_topic = '/' + self.ego_id + '/opp_odom'
            self.opp_ego_odom_topic = '/' + self.opp_id + '/opp_odom'
            self.opp_drive_topic = '/' + self.opp_id + '/drive'
        else:
            # single car timed trial (qualifying)
            pass
        self.scan_distance_to_base_link = rospy.get_param('scan_distance_to_base_link')

        # Map
        self.map_path = os.environ.get('RACE_MAP_PATH')
        self.map_img_ext = os.environ.get('RACE_MAP_IMG_EXT')
        # self.map_path = rospy.get_param('map_path')
        # self.map_img_ext = rospy.get_param('map_img_ext')

        # C++ backend
        exec_dir = os.environ.get('F1TENTH_EXEC_DIR')
        # exec_dir = rospy.get_param('executable_dir')

        # Scan simulation params
        scan_fov = rospy.get_param('scan_fov')
        scan_beams = rospy.get_param('scan_beams')
        self.angle_min = -scan_fov / 2.
        self.angle_max = scan_fov / 2.
        self.angle_inc = scan_fov / scan_beams

        # Track centerline, legacy
        # csv_path = rospy.get_param('waypoints_path')

        # Vehicle parameters
        wheelbase = 0.3302
        mass = 3.74
        l_r = 0.17145
        I_z = 0.04712
        mu = 0.523
        h_cg = 0.074
        cs_f = 4.718
        cs_r = 5.4562

        # init gym backend
        self.racecar_env = gym.make('f110_gym:f110-v0')
        self.racecar_env.init_map(self.map_path, self.map_img_ext, False, False)
        self.racecar_env.update_params(mu, h_cg, l_r, cs_f, cs_r, I_z, mass, exec_dir, double_finish=True)

        driver_count = 2 if self.race_scenario > 0 else 1
        poses = [[0. + (i * 0.75), 0. - (i*1.5), np.radians(60)] for i in range(driver_count)]
        if driver_count == 1:
            poses.append([200.0, 200.0, 0.0])

        # calculate initial state
        initial_state = {
            'x': [poses[0][0], poses[1][0]],
            'y': [poses[0][1], poses[1][1]],
            'theta': [poses[0][2], poses[1][2]],
        }
        self.ego_pose = poses[0]
        self.opp_pose = poses[1]

        self.obs, _, self.done, _ = self.racecar_env.reset(initial_state)

        self.ego_speed = [0., 0., 0.]
        self.ego_steer = 0.0
        self.ego_requested_speed = 0.0
        self.ego_drive_published = False

        self.opp_speed = [0., 0., 0.]
        self.opp_steer = 0.0
        self.opp_requested_speed = 0.0
        self.opp_drive_published = False

        # keep track of latest sim state
        self.ego_scan = list(self.obs['scans'][0])
        self.opp_scan = list(self.obs['scans'][1])

        # keep track of collision
        self.ego_collision = False
        self.opp_collision = False

        # transform broadcaster
        self.br = transform_broadcaster.TransformBroadcaster()

        # publishers
        if self.race_scenario:
            self.ego_scan_pub = rospy.Publisher(self.ego_scan_topic, LaserScan, queue_size=1)
            self.opp_scan_pub = rospy.Publisher(self.opp_scan_topic, LaserScan, queue_size=1)
            self.ego_odom_pub = rospy.Publisher(self.ego_odom_topic, Odometry, queue_size=1)
            self.ego_opp_odom_pub = rospy.Publisher(self.ego_opp_odom_topic, Odometry, queue_size=1)
            self.opp_odom_pub = rospy.Publisher(self.opp_odom_topic, Odometry, queue_size=1)
            self.opp_ego_odom_pub = rospy.Publisher(self.opp_ego_odom_topic, Odometry, queue_size=1)
            self.info_pub = rospy.Publisher(self.race_info_topic, RaceInfo, queue_size=1)
        else:
            self.ego_scan_pub = rospy.Publisher(self.ego_scan_topic, LaserScan, queue_size=1)
            self.ego_odom_pub = rospy.Publisher(self.ego_odom_topic, Odometry, queue_size=1)
            self.opp_odom_pub = rospy.Publisher(self.opp_odom_topic, Odometry, queue_size=1)
            self.info_pub = rospy.Publisher(self.race_info_topic, RaceInfo, queue_size=1)

        # subs
        self.drive_sub = rospy.Subscriber(self.ego_drive_topic, AckermannDriveStamped, self.drive_callback,
                                          queue_size=1)
        if self.race_scenario:
            self.opp_drive_sub = rospy.Subscriber(self.opp_drive_topic, AckermannDriveStamped, self.opp_drive_callback,
                                                  queue_size=1)
        # Timer
        self.timer = rospy.Timer(rospy.Duration(0.004), self.timer_callback)
        self.drive_timer = rospy.Timer(rospy.Duration(0.01), self.drive_timer_callback)

    def update_sim_state(self):
        self.ego_scan = list(self.obs['scans'][0])
        if self.race_scenario:
            self.opp_scan = list(self.obs['scans'][1])

        self.ego_pose[0] = self.obs['poses_x'][0]
        self.ego_pose[1] = self.obs['poses_y'][0]
        self.ego_pose[2] = self.obs['poses_theta'][0]
        self.ego_speed[0] = self.obs['linear_vels_x'][0]
        self.ego_speed[1] = self.obs['linear_vels_y'][0]
        self.ego_speed[2] = self.obs['ang_vels_z'][0]

        self.opp_pose[0] = self.obs['poses_x'][1]
        self.opp_pose[1] = self.obs['poses_y'][1]
        self.opp_pose[2] = self.obs['poses_theta'][1]
        self.opp_speed[0] = self.obs['linear_vels_x'][1]
        self.opp_speed[1] = self.obs['linear_vels_y'][1]
        self.opp_speed[2] = self.obs['ang_vels_z'][1]

    def drive_callback(self, drive_msg):
        self.ego_requested_speed = drive_msg.drive.speed
        self.ego_steer = drive_msg.drive.steering_angle
        self.ego_drive_published = True

    def opp_drive_callback(self, opp_drive_msg):
        self.opp_requested_speed = opp_drive_msg.drive.speed
        self.opp_steer = opp_drive_msg.drive.steering_angle
        self.opp_drive_published = True

    def drive_timer_callback(self, timer):
        if self.race_scenario:
            # two car
            if self.ego_drive_published and self.opp_drive_published:
                action = {'ego_idx': 0, 'speed': [self.ego_requested_speed, self.opp_requested_speed],
                          'steer': [self.ego_steer, self.opp_steer]}
                self.obs, step_reward, self.done, info = self.racecar_env.step(action)
                self.update_sim_state()
        else:
            # single car
            if self.ego_drive_published:
                action = {'ego_idx': 0, 'speed': [self.ego_requested_speed, 0.], 'steer': [self.ego_steer, 0.]}
                self.obs, step_reward, self.done, info = self.racecar_env.step(action)
                self.update_sim_state()

    def timer_callback(self, timer):
        ts = rospy.Time.now()

        # pub ego scan
        scan = LaserScan()
        scan.header.stamp = ts
        scan.header.frame_id = 'ego_racecar/laser'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = 0.
        scan.range_max = 30.
        scan.ranges = self.ego_scan
        self.ego_scan_pub.publish(scan)

        # conditionally pub opp scan
        if self.race_scenario:
            opp_scan = LaserScan()
            opp_scan.header.stamp = ts
            opp_scan.header.frame_id = 'opp_racecar/laser'
            opp_scan.angle_min = self.angle_min
            opp_scan.angle_max = self.angle_max
            opp_scan.angle_increment = self.angle_inc
            opp_scan.range_min = 0.
            opp_scan.range_max = 30.
            opp_scan.ranges = self.opp_scan
            self.opp_scan_pub.publish(opp_scan)

        # pub tf
        self.publish_odom(ts)
        self.publish_transforms(ts)
        self.publish_laser_transforms(ts)
        self.publish_wheel_transforms(ts)

        # pub race info
        self.publish_race_info(ts)

    def publish_race_info(self, ts):
        info = RaceInfo()
        info.header.stamp = ts
        if not self.ego_collision:
            self.ego_collision = self.obs['collisions'][0]
        if not self.opp_collision:
            self.opp_collision = self.obs['collisions'][1]
        info.ego_collision = self.ego_collision
        info.opp_collision = self.opp_collision
        info.ego_elapsed_time = self.obs['lap_times'][0]
        info.opp_elapsed_time = self.obs['lap_times'][1]
        info.ego_lap_count = self.obs['lap_counts'][0]
        info.opp_lap_count = self.obs['lap_counts'][1]
        self.info_pub.publish(info)

    def publish_odom(self, ts):
        ego_odom = Odometry()
        ego_odom.header.stamp = ts
        ego_odom.header.frame_id = '/map'
        ego_odom.child_frame_id = 'ego_racecar/base_link'
        ego_odom.pose.pose.position.x = self.ego_pose[0]
        ego_odom.pose.pose.position.y = self.ego_pose[1]
        ego_quat = quaternion_from_euler(0., 0., self.ego_pose[2])
        ego_odom.pose.pose.orientation.x = ego_quat[0]
        ego_odom.pose.pose.orientation.y = ego_quat[1]
        ego_odom.pose.pose.orientation.z = ego_quat[2]
        ego_odom.pose.pose.orientation.w = ego_quat[3]
        ego_odom.twist.twist.linear.x = self.ego_speed[0]
        ego_odom.twist.twist.linear.y = self.ego_speed[1]
        ego_odom.twist.twist.angular.z = self.ego_speed[2]
        self.ego_odom_pub.publish(ego_odom)
        if self.race_scenario:
            self.opp_ego_odom_pub.publish(ego_odom)

        opp_odom = Odometry()
        opp_odom.header.stamp = ts
        opp_odom.header.frame_id = '/map'
        opp_odom.child_frame_id = 'opp_racecar/base_link'
        opp_odom.pose.pose.position.x = self.opp_pose[0]
        opp_odom.pose.pose.position.y = self.opp_pose[1]
        opp_quat = quaternion_from_euler(0., 0., self.opp_pose[2])
        opp_odom.pose.pose.orientation.x = opp_quat[0]
        opp_odom.pose.pose.orientation.y = opp_quat[1]
        opp_odom.pose.pose.orientation.z = opp_quat[2]
        opp_odom.pose.pose.orientation.w = opp_quat[3]
        opp_odom.twist.twist.linear.x = self.opp_speed[0]
        opp_odom.twist.twist.linear.y = self.opp_speed[1]
        opp_odom.twist.twist.angular.z = self.opp_speed[2]
        self.opp_odom_pub.publish(opp_odom)
        if self.race_scenario:
            self.ego_opp_odom_pub.publish(opp_odom)

    def publish_transforms(self, ts):
        ego_t = Transform()
        ego_t.translation.x = self.ego_pose[0]
        ego_t.translation.y = self.ego_pose[1]
        ego_t.translation.z = 0.0
        ego_quat = quaternion_from_euler(0.0, 0.0, self.ego_pose[2])
        ego_t.rotation.x = ego_quat[0]
        ego_t.rotation.y = ego_quat[1]
        ego_t.rotation.z = ego_quat[2]
        ego_t.rotation.w = ego_quat[3]

        ego_ts = TransformStamped()
        ego_ts.transform = ego_t
        ego_ts.header.stamp = ts
        ego_ts.header.frame_id = '/map'
        ego_ts.child_frame_id = 'ego_racecar/base_link'

        opp_t = Transform()
        opp_t.translation.x = self.opp_pose[0]
        opp_t.translation.y = self.opp_pose[1]
        opp_t.translation.z = 0.0
        opp_quat = quaternion_from_euler(0.0, 0.0, self.opp_pose[2])
        opp_t.rotation.x = opp_quat[0]
        opp_t.rotation.y = opp_quat[1]
        opp_t.rotation.z = opp_quat[2]
        opp_t.rotation.w = opp_quat[3]

        opp_ts = TransformStamped()
        opp_ts.transform = opp_t
        opp_ts.header.stamp = ts
        opp_ts.header.frame_id = '/map'
        opp_ts.child_frame_id = 'opp_racecar/base_link'

        self.br.sendTransform(ego_ts)
        self.br.sendTransform(opp_ts)

    def publish_wheel_transforms(self, ts):
        ego_wheel_ts = TransformStamped()
        ego_wheel_quat = quaternion_from_euler(0., 0., self.ego_steer)
        ego_wheel_ts.transform.rotation.x = ego_wheel_quat[0]
        ego_wheel_ts.transform.rotation.y = ego_wheel_quat[1]
        ego_wheel_ts.transform.rotation.z = ego_wheel_quat[2]
        ego_wheel_ts.transform.rotation.w = ego_wheel_quat[3]
        ego_wheel_ts.header.stamp = ts
        ego_wheel_ts.header.frame_id = 'ego_racecar/front_left_hinge'
        ego_wheel_ts.child_frame_id = 'ego_racecar/front_left_wheel'
        self.br.sendTransform(ego_wheel_ts)
        ego_wheel_ts.header.frame_id = 'ego_racecar/front_right_hinge'
        ego_wheel_ts.child_frame_id = 'ego_racecar/front_right_wheel'
        self.br.sendTransform(ego_wheel_ts)

        opp_wheel_ts = TransformStamped()
        opp_wheel_quat = quaternion_from_euler(0., 0., self.opp_steer)
        opp_wheel_ts.transform.rotation.x = opp_wheel_quat[0]
        opp_wheel_ts.transform.rotation.y = opp_wheel_quat[1]
        opp_wheel_ts.transform.rotation.z = opp_wheel_quat[2]
        opp_wheel_ts.transform.rotation.w = opp_wheel_quat[3]
        opp_wheel_ts.header.stamp = ts
        opp_wheel_ts.header.frame_id = 'opp_racecar/front_left_hinge'
        opp_wheel_ts.child_frame_id = 'opp_racecar/front_left_wheel'
        self.br.sendTransform(opp_wheel_ts)
        opp_wheel_ts.header.frame_id = 'opp_racecar/front_right_hinge'
        opp_wheel_ts.child_frame_id = 'opp_racecar/front_right_wheel'
        self.br.sendTransform(opp_wheel_ts)

    def publish_laser_transforms(self, ts):
        ego_scan_ts = TransformStamped()
        ego_scan_ts.transform.translation.x = self.scan_distance_to_base_link
        ego_scan_ts.transform.rotation.w = 1.
        ego_scan_ts.header.stamp = ts
        # TODO: check frame names
        ego_scan_ts.header.frame_id = 'ego_racecar/base_link'
        ego_scan_ts.child_frame_id = 'ego_racecar/laser'
        self.br.sendTransform(ego_scan_ts)

        opp_scan_ts = TransformStamped()
        opp_scan_ts.transform.translation.x = self.scan_distance_to_base_link
        opp_scan_ts.transform.rotation.w = 1.
        opp_scan_ts.header.stamp = ts
        # TODO: check frame names
        opp_scan_ts.header.frame_id = 'opp_racecar/base_link'
        opp_scan_ts.child_frame_id = 'opp_racecar/laser'
        self.br.sendTransform(opp_scan_ts)


if __name__ == '__main__':
    rospy.init_node('gym_bridge')
    gym_bridge = GymBridge()
    rospy.spin()
