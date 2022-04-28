ARG PARENT
FROM ${PARENT}

COPY ./compose/ros.asc /ros.asc
RUN sudo apt-key add /ros.asc

RUN apt-get update --fix-missing
RUN apt-get install -y libzmq3-dev \
                       git \
                       build-essential \
                       autoconf \
                       libtool \
                       libeigen3-dev \
                       cmake \
                       ros-${ROS_DISTRO}-ackermann-msgs \
                       ros-${ROS_DISTRO}-map-server \
                       ros-${ROS_DISTRO}-genpy \
                       ros-${ROS_DISTRO}-rviz

RUN apt-get install -y python3-pip
RUN pip3 install -U pip

# Install F1Tenth GYM
#RUN git clone -b cpp_backend_archive --depth=1 https://github.com/f1tenth/f1tenth_gym /f1tenth_gym
#RUN cd f1tenth_gym && mkdir build && cd build && cmake .. && make
#RUN cd f1tenth_gym && cp ./build/sim_requests_pb2.py ./gym/ && pip install -e gym/
COPY ./gym /f1tenth/gym
RUN pip3 install -e /f1tenth/gym/

# Install F1Tenth GYM Bridge
RUN mkdir -p /catkin_ws/src
COPY ./f1tenth_gym_ros /catkin_ws/src/f1tenth_gym_ros
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; cd catkin_ws; catkin_make; source devel/setup.bash"

ENV F1TENTH_EXEC_DIR=/f1tenth_gym/build/

COPY ./compose/bridge/start.sh /start.sh
RUN chmod +x /start.sh
CMD ["/start.sh"]