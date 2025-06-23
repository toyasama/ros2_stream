FROM ros:humble

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-rmw-cyclonedds-cpp \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ffmpeg \
    ros-humble-image-transport-plugins \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y \
    wget && \
    rm -rf /var/lib/apt/lists/*
    
RUN wget https://github.com/bluenviron/mediamtx/releases/download/v1.11.1/mediamtx_v1.11.1_linux_amd64.tar.gz -O /tmp/mediamtx.tar.gz \
    && tar -xzf /tmp/mediamtx.tar.gz -C /usr/local/bin \
    && rm /tmp/mediamtx.tar.gz \
    && chmod +x /usr/local/bin/mediamtx
    
ARG USERNAME=devuser
RUN useradd -ms /bin/bash $USERNAME  && usermod -aG video $USERNAME
USER $USERNAME
WORKDIR /home/$USERNAME

RUN mkdir -p /home/$USERNAME/workspace/src
COPY --chown=$USERNAME:$USERNAME ../src /home/$USERNAME/workspace/src

COPY cyclonedds.xml /etc/cyclonedds.xml
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "[ -f /home/$USERNAME/workspace/install/setup.bash ] && source /home/$USERNAME/workspace/install/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc && \
    echo "export CYCLONEDDS_URI=file:///etc/cyclonedds.xml" >> ~/.bashrc

COPY setup.bash /setup.bash
USER root
RUN chmod +x /setup.bash
RUN echo "source /setup.bash" >> /home/$USERNAME/.bashrc
USER $USERNAME

WORKDIR /home/$USERNAME/workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch ros_stream ros_stream_launch.py"]
