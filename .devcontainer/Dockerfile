# Official ROS2 Humble image with desktop support
FROM osrf/ros:humble-desktop

# Python and system dependencies
RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    git \
    build-essential \
    ros-humble-rmw-cyclonedds-cpp \
    sudo && \
    rm -rf /var/lib/apt/lists/*

# Extra dependencies
RUN apt-get update && apt-get install -y \
    ffmpeg \
    ros-humble-image-transport-plugins \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# Install MediaMTX (RTSP server)
RUN apt-get update && \
    apt-get install -y \
    wget && \
    rm -rf /var/lib/apt/lists/*
    
RUN wget https://github.com/bluenviron/mediamtx/releases/download/v1.11.1/mediamtx_v1.11.1_linux_amd64.tar.gz -O /tmp/mediamtx.tar.gz \
    && tar -xzf /tmp/mediamtx.tar.gz -C /usr/local/bin \
    && rm /tmp/mediamtx.tar.gz \
    && chmod +x /usr/local/bin/mediamtx
    
# Create non-root user for development
ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME &&\
    usermod -aG video devuser

USER $USERNAME
WORKDIR /home/$USERNAME

# Improve shell prompt and history behavior
RUN sed -i 's/^#force_color_prompt=yes/force_color_prompt=yes/' ~/.bashrc && \
    echo 'HISTFILE=~/.bash_history' >> ~/.bashrc && \
    echo 'HISTSIZE=1000' >> ~/.bashrc && \
    echo 'HISTFILESIZE=2000' >> ~/.bashrc && \
    echo 'shopt -s histappend' >> ~/.bashrc && \
    echo 'PROMPT_COMMAND="history -a; $PROMPT_COMMAND"' >> ~/.bashrc

# Configure ROS2 and Cyclone DDS environment
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

CMD ["sleep", "infinity"]
