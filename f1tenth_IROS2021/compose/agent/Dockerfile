# Base image
ARG PARENT
FROM ${PARENT}

COPY ./compose/ros.asc /ros.asc
RUN sudo apt-key add /ros.asc

RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-ackermann-msgs

# Creating a catkin workspace
RUN mkdir -p /catkin_ws/src

# Copying
COPY ./pkg /catkin_ws/src/pkg
RUN chmod +x /catkin_ws/src/pkg/nodes/f1tenth_ros_agent.py

## Building your ROS packages
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; cd catkin_ws; catkin_make; source devel/setup.bash"

# Uncomment set the shell to bash to provide the "source" command
SHELL ["/bin/bash", "-c"]

# Setting entry point command that runs when the container is brought up
CMD source /catkin_ws/devel/setup.bash; roslaunch --wait pkg f1tenth_gym_agent.launch
