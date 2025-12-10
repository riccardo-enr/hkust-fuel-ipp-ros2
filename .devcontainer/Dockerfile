FROM osrf/ros:jazzy-desktop-full

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # Build tools
    build-essential \
    cmake \
    git \
    wget \
    # Required libraries from README
    libarmadillo-dev \
    libpcl-dev \
    libeigen3-dev \
    # ROS 2 packages
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf2-eigen \
    ros-jazzy-visualization-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    # Colcon build tools
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    # Development utilities
    gdb \
    vim \
    nano \
    bash-completion \
    && rm -rf /var/lib/apt/lists/*

# Install NLopt v2.7.1 as specified in README
RUN cd /tmp && \
    git clone -b v2.7.1 https://github.com/stevengj/nlopt.git && \
    cd nlopt && \
    mkdir -p build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd / && \
    rm -rf /tmp/nlopt

# Initialize rosdep
RUN rosdep init || true && \
    rosdep update

# Create a non-root user
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Switch to non-root user
USER $USERNAME

# Setup bash environment for the user
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc && \
    echo "# Source workspace overlay (uncomment after first build)" >> /home/$USERNAME/.bashrc && \
    echo "# source /workspace/install/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "" >> /home/$USERNAME/.bashrc && \
    echo "# FUEL workspace aliases" >> /home/$USERNAME/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> /home/$USERNAME/.bashrc && \
    echo "alias ct='colcon test'" >> /home/$USERNAME/.bashrc && \
    echo "alias cbp='colcon build --symlink-install --packages-select'" >> /home/$USERNAME/.bashrc && \
    echo "alias cbs='colcon build --symlink-install --packages-up-to'" >> /home/$USERNAME/.bashrc && \
    echo "alias cclean='rm -rf build install log'" >> /home/$USERNAME/.bashrc

# Set working directory
WORKDIR /workspace

# Reset DEBIAN_FRONTEND
ENV DEBIAN_FRONTEND=dialog

# Source ROS 2 setup in entrypoint
CMD ["/bin/bash"]
