#!/bin/bash
set -e

# Detect if we are launching zsh or bash
EXT="bash"
if [[ "$@" == *"zsh"* ]]; then
    EXT="zsh"
fi

source_if_exists() {
  local setup_file="$1"
  if [ -f "${setup_file}" ]; then
    source "${setup_file}"
  fi
}

source_if_exists "/opt/ros/jazzy/setup.$EXT"
source_if_exists "/workspace/install/setup.$EXT"

# Source nested workspaces
for ws_install in /workspace/*/install; do
  if [ -d "${ws_install}" ] && [ "${ws_install}" != "/workspace/install" ]; then
    source_if_exists "${ws_install}/setup.$EXT"
  fi
done

exec "$@"
