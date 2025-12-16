# ROS 1 → ROS 2 Checklist

Use this checklist per package.

- [ ] Update `package.xml` to format 3 with `ament_cmake` dependencies.
- [ ] Replace `CMakeLists.txt` Catkin macros with `ament_cmake` equivalents.
- [ ] Port all nodes to `rclcpp`, replacing `ros::NodeHandle` usage.
- [ ] Swap TF (`tf`) for `tf2` + `tf2_ros`.
- [ ] Convert launch files to Python API (`launch_ros.actions.Node`).
- [ ] Ensure parameters are declared with defaults.
- [ ] Add or update tests via `ament_add_gtest` or launch tests.
- [ ] Document the migration in `docs/porting/PORTING_PROGRESS.md`.
