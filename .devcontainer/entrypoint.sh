#!/bin/bash
set -e

source_if_exists() {
  local setup_file="$1"
  if [ -f "${setup_file}" ]; then
    source "${setup_file}"
  fi
}

source_if_exists "/opt/ros/jazzy/setup.bash"
source_if_exists "/workspace/install/setup.bash"

# Source nested workspaces
for ws_install in /workspace/*/install; do
  if [ -d "${ws_install}" ] && [ "${ws_install}" != "/workspace/install" ]; then
    source_if_exists "${ws_install}/setup.bash"
  fi
done

# Ensure zshrc sources ROS 2 setup if zsh is being executed
if [[ "$*" == *"zsh"* ]]; then
  ZSHRC="$HOME/.zshrc"
  if [ -f "$ZSHRC" ]; then
    if ! grep -q "source /opt/ros/jazzy/setup.zsh" "$ZSHRC"; then
      echo "Configuring zshrc for ROS 2..."
      {
        echo ""
        echo "# Auto-added by entrypoint.sh"
        echo "source /opt/ros/jazzy/setup.zsh"
        echo "if [ -f /workspace/install/setup.zsh ]; then source /workspace/install/setup.zsh; fi"
      } >> "$ZSHRC"
    fi
  fi
fi

exec "$@"
