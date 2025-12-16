# Dev Container Setup

1. `docker compose exec fuel-dev bash`
2. `source /opt/ros/jazzy/setup.bash`
3. `colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo`
4. `source install/setup.bash`

_Add troubleshooting tips for volume mounts, permissions, and GPU passthrough here._
