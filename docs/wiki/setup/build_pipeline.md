# Build & Test Pipeline

Document the required commands, order, and environment variables to keep overlays consistent.

## Core Commands
- `colcon build --packages-select plan_manage`
- `colcon build --packages-select plan_manage poly_traj uav_utils`
- `colcon test --event-handlers console_direct+`
- `colcon test --packages-select poly_traj --ctest-args -R traj_generator`

## Notes
- Always source `install/setup.bash` after building.
- Keep `CMAKE_EXPORT_COMPILE_COMMANDS` enabled for tooling.
- Mention any required `AMENT_PREFIX_PATH` or `RMW_IMPLEMENTATION` overrides.
