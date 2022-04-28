#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash

# start roscore & rviz
nohup roscore > /tmp/roscore.log &

tail -f /dev/null;