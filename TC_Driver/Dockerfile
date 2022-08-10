FROM ros:noetic-ros-base

RUN apt-get update && apt-get install -y python3-catkin-tools ros-noetic-rospy

ADD ./tc_driver_ros /catkin_wssrc?tc_driver_ros
ADD ./requirements_inference.txt ./requirements.txt

# install python packages 
RUN pip install -r requirements.txt

WORKDIR /catkin_ws

RUN /bin/bash -c 'find . -name "*.py" -exec chmod +x {} \;'
#RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin build'
RUN /bin/bash -c 'catkin build'
RUN /bin/bash -c 'source /catkin_ws/devel/setup.bash'

