#!/bin/bash
set -e
shopt -s nullglob

source_if_exists() {
  local setup_file="$1"
  if [ -f "${setup_file}" ]; then
    # shellcheck disable=SC1090
    source "${setup_file}"
  fi
}

# Source ROS 2
source_if_exists /opt/ros/jazzy/setup.bash

# Source the primary workspace (if present)
source_if_exists /workspace/install/setup.bash

# Automatically source any nested workspaces under /workspace/*/install
for ws_install in /workspace/*/install; do
  if [ -d "${ws_install}" ] && [ "${ws_install}" != "/workspace/install" ]; then
    source_if_exists "${ws_install}/setup.bash"
  fi
done

exec "$@"
