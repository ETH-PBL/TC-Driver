#!/usr/bin/env python3
import rospy
import os
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

from f1tenth_gym_ros.msg import RaceInfo

current_dir = os.path.abspath(os.path.dirname(__file__))
package_dir = os.path.abspath(os.path.join(current_dir, ".."))

import numpy as np
import gym
import time


class Agent(object):
    def __init__(self, id, drive_callback):
        self.id = id
        self.scan_topic = '/%s/scan' % self.id
        self.drive_topic = '/%s/drive' % self.id
        self.collision = False
        self.requested_steer = 0.0
        self.requested_speed = 0.0
        self.drive_published = False
        self.scan = False
        self.scan_pub = rospy.Publisher(self.scan_topic, LaserScan, queue_size=1)
        self.drive_sub = rospy.Subscriber(self.drive_topic, AckermannDriveStamped, drive_callback, queue_size=1)


class GymBridge(object):

    def __init__(self):
        # get env vars
        self.race_scenario = int(os.environ.get('RACE_SCENARIO', 0))
        self.agents = []

        # this is filled when match is finished
        self.info = {}

        self.agents.append(Agent(os.environ.get("EGO_ID"), self.drive_callback))
        if self.race_scenario > 0:
            self.agents.append(Agent(os.environ.get('OPP_ID'), self.opp_drive_callback))

        # Topic Names
        self.race_info_topic = rospy.get_param('race_info_topic')

        # Map
        self.map_path = os.environ.get('RACE_MAP_PATH')
        self.map_img_ext = os.environ.get('RACE_MAP_IMG_EXT')

        # Scan simulation params
        scan_fov = rospy.get_param('scan_fov')
        scan_beams = rospy.get_param('scan_beams')
        self.angle_min = -scan_fov / 2.
        self.angle_max = scan_fov / 2.
        self.angle_inc = scan_fov / scan_beams

        # publishers
        self.info_pub = rospy.Publisher(self.race_info_topic, RaceInfo, queue_size=1)

        # Launch
        driver_count = 2 if self.race_scenario > 0 else 1
        self.env = gym.make('f110_gym:f110-v0',
                            map=self.map_path[:-5],
                            map_ext=self.map_img_ext, num_agents=driver_count)

        # init gym backend
        poses = [[0. + (i * 0.75), 0. - (i*1.5), np.radians(60)] for i in range(driver_count)]
        self.obs, _, self.done, _ = self.env.reset(poses=np.array(poses))

        if os.environ.get("DISPLAY"):
            self.env.render()

        self.update_sim_state()

        # Timer
        self.timer = rospy.Timer(rospy.Duration(0.004), self.timer_callback)
        self.drive_timer = rospy.Timer(rospy.Duration(0.02), self.drive_timer_callback)

    def spin(self):
        print("Starting F1Tenth Bridge")

        # run until challenge completes or ros shuts down
        while not self.done and not rospy.core.is_shutdown():
            rospy.rostime.wallsleep(0.04)

            if os.environ.get("DISPLAY"):
                self.env.render()

        print("Shutting down F1Tenth Bridge")

    def update_sim_state(self):
        for i, scan in enumerate(self.obs['scans']):
            self.agents[i].scan = scan

        for i, collision in enumerate(self.obs['collisions']):
            self.agents[i].collision = bool(collision)

    def drive_callback(self, drive_msg):
        self.agents[0].requested_speed = drive_msg.drive.speed
        self.agents[0].requested_steer = drive_msg.drive.steering_angle
        self.agents[0].drive_published = True

    def opp_drive_callback(self, drive_msg):
        self.agents[1].requested_speed = drive_msg.drive.speed
        self.agents[1].requested_steer = drive_msg.drive.steering_angle
        self.agents[1].drive_published = True

    def drive_timer_callback(self, timer):
        published = all([a.drive_published for a in self.agents])

        # until all agents started publishing, we wait
        if not published:
            return

        # update simulation
        actions = [[a.requested_steer, a.requested_speed] for a in self.agents]
        self.obs, _, self.done, _ = self.env.step(np.array(actions))

        # update scan data
        self.update_sim_state()

        # if match is completed, we set the bridge.info property
        if self.done:
            info = {
                "ego_collision": self.agents[0].collision,
                "ego_elapsed_time": float(self.obs['lap_times'][0]),
                "ego_lap_count": float(self.obs['lap_counts'][0]),
            }

            if len(self.agents) > 1:
                info["opp_collision"] = self.agents[1].collision
                info["opp_elapsed_time"] = float(self.obs['lap_times'][1])
                info["opp_lap_count"] = float(self.obs['lap_counts'][1])
            self.info = info

    def timer_callback(self, timer):
        # once match is completed, stop publishing information
        if self.done:
            return

        ts = rospy.Time.now()

        def generate_scan_message(name, ranges):
            scan = LaserScan()
            scan.header.stamp = ts
            scan.header.frame_id = '%s/laser' % name
            scan.angle_min = self.angle_min
            scan.angle_max = self.angle_max
            scan.angle_increment = self.angle_inc
            scan.range_min = 0.
            scan.range_max = 30.
            scan.ranges = ranges
            return scan

        names = ["ego_racecar", "opp_racecar"]

        for i, agent in enumerate(self.agents):
            scan = generate_scan_message(names[i], agent.scan)
            try:
                agent.scan_pub.publish(scan)
            except Exception as e:
                pass

        # pub race info
        self.publish_race_info(ts)

    def publish_race_info(self, ts):
        info = RaceInfo()
        info.header.stamp = ts
        info.ego_collision = self.agents[0].collision
        info.ego_elapsed_time = self.obs['lap_times'][0]
        info.ego_lap_count = self.obs['lap_counts'][0]
        info.opp_collision = self.agents[1].collision if len(self.agents) > 1 else 0
        info.opp_elapsed_time = self.obs['lap_times'][1] if len(self.agents) > 1 else 0.0
        info.opp_lap_count = self.obs['lap_counts'][1] if len(self.agents) > 1 else 0
        self.info_pub.publish(info)


if __name__ == '__main__':
    rospy.init_node('gym_bridge')
    gym_bridge = GymBridge()
    gym_bridge.spin()

    # once we're here, we know that competition is completed, so we publish info to API
    print (gym_bridge.info)

    time.sleep(1)
