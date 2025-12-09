FROM ros:jazzy

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    wget \
    build-essential \
    cmake \
    libarmadillo-dev \
    python3-colcon-common-extensions \
    python3-pip \
    sudo \
    ros-jazzy-desktop \
    && rm -rf /var/lib/apt/lists/*

# Install nlopt v2.7.1 from source as per README
WORKDIR /tmp
RUN git clone -b v2.7.1 https://github.com/stevengj/nlopt.git \
    && cd nlopt \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install \
    && cd /tmp \
    && rm -rf nlopt

# Set up a user (optional but recommended for devcontainers)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN userdel -r ubuntu || true \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Source ROS setup in .bashrc for the user
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc

ENV SHELL=/bin/bash
USER $USERNAME
WORKDIR /home/$USERNAME/ws