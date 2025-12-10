#!/bin/bash
set -e

echo "Setting up FUEL ROS 2 Jazzy development environment..."

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Update package lists
sudo apt-get update

# Install required dependencies from README
echo "Installing Armadillo..."
sudo apt-get install -y libarmadillo-dev

# Install PCL (Point Cloud Library) - required by the project
echo "Installing PCL..."
sudo apt-get install -y libpcl-dev

# Install Eigen3 - common dependency for robotics projects
echo "Installing Eigen3..."
sudo apt-get install -y libeigen3-dev

# Install additional ROS 2 packages that might be needed
echo "Installing additional ROS 2 packages..."
sudo apt-get install -y \
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
    ros-jazzy-rqt-common-plugins

# Install NLopt v2.7.1 as specified in README
echo "Installing NLopt v2.7.1..."
cd /tmp
if [ ! -d "nlopt" ]; then
    git clone -b v2.7.1 https://github.com/stevengj/nlopt.git
fi
cd nlopt
mkdir -p build
cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
cd /workspace

# Install colcon build tools and utilities
echo "Installing colcon build tools..."
sudo apt-get install -y \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep

# Initialize rosdep if not already initialized
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "Initializing rosdep..."
    sudo rosdep init
fi
rosdep update

# Setup workspace
echo "Setting up workspace..."
cd /workspace

# Add ROS 2 sourcing to bashrc for convenience
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

# Add colcon autocomplete
if ! grep -q "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ~/.bashrc; then
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
fi

# Add workspace overlay to bashrc (will be created after first build)
if ! grep -q "source /workspace/install/setup.bash" ~/.bashrc; then
    echo "# Source workspace overlay (uncomment after first build)" >> ~/.bashrc
    echo "# source /workspace/install/setup.bash" >> ~/.bashrc
fi

# Create useful aliases
if ! grep -q "alias cb='colcon build" ~/.bashrc; then
    cat >> ~/.bashrc << 'EOF'

# FUEL workspace aliases
alias cb='colcon build --symlink-install'
alias ct='colcon test'
alias cbp='colcon build --symlink-install --packages-select'
alias cbs='colcon build --symlink-install --packages-up-to'
alias cclean='rm -rf build install log'

EOF
fi

# Clean up
sudo apt-get clean
sudo rm -rf /var/lib/apt/lists/*

echo "========================================"
echo "Setup complete!"
echo "========================================"
echo ""
echo "To build the workspace, run:"
echo "  colcon build --symlink-install"
echo ""
echo "Or use the alias:"
echo "  cb"
echo ""
echo "After building, source the workspace:"
echo "  source install/setup.bash"
echo ""
echo "Happy coding! 🚀"
