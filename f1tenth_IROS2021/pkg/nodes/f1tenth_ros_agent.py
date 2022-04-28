#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import numpy as np
import os

from pkg.drivers import DisparityExtender as Driver

"""
NOTE: Following code enables F1Tenth - Docker - ROS integration.  
Please don't change it, unless you know what you're doing
"""


class ROSRunner:
    def __init__(self, driver, agent_name):
        self.driver = driver
        self.agent_name = agent_name
        self.pub_drive = None

    def lidar_callback(self, data):
        ranges = np.asarray(data.ranges)
        speed, angle = self.driver.process_lidar(ranges)

        # create message & publish
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        self.pub_drive.publish(msg)

    def run(self):
        print ("agent_name", agent_name)

        rospy.init_node('gym_agent_%s' % self.agent_name, anonymous=True)
        self.pub_drive = rospy.Publisher('/%s/drive' % self.agent_name, AckermannDriveStamped, queue_size=5)

        # start listening
        rospy.Subscriber('/%s/scan' % self.agent_name, LaserScan, self.lidar_callback)
        rospy.sleep(3)
        rospy.spin()


if __name__ == "__main__":
    agent_name = os.environ.get("F1TENTH_AGENT_NAME")
    runner = ROSRunner(Driver(), agent_name)
    # launch
    runner.run()
