#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Source the workspace if it exists
if [ -f "/workspace/install/setup.bash" ]; then
  source /workspace/install/setup.bash
fi

exec "$@"
