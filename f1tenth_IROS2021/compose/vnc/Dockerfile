ARG PARENT
ARG ROS_DISTRO
FROM ${PARENT} as with-vnc

    ENV RIDERS_PATH=/opt/riders
    ENV ROS_DISTRO=${ROS_DISTRO}
    
    RUN mkdir -p ${RIDERS_PATH}
    
    # Install Xvfb, JavaFX-helpers and Openbox window manager
    RUN apt-get update 
    RUN sudo apt-get install -yq xvfb x11vnc xterm openjfx libopenjfx-java \
        openbox libx11-dev libxkbfile-dev libxkbfile-dev git && \
        sudo rm -rf /var/lib/apt/lists/*

    # Install novnc
    RUN git clone https://github.com/novnc/noVNC.git /opt/novnc \
        && git clone https://github.com/novnc/websockify /opt/novnc/utils/websockify

    # Copy openbox configuration file to home
    # TODO: Should we move these to final image?
    COPY ./compose/vnc/rc.xml /root/.config/openbox/rc.xml
    COPY ./compose/vnc/index.html /opt/novnc/index.html
    COPY ./compose/vnc/vnc.html /opt/novnc/vnc.html
    COPY ./compose/vnc/setup.sh ${RIDERS_PATH}/setup.sh