# Porting Playbook

Use this as a repeatable recipe when touching packages that are still on ROS 1.

1. **Inventory** – run `rg "ros::" src/<pkg>` to find ROS 1 APIs.
2. **Manifests** – confirm dependencies + export tags match ROS 2 expectations.
3. **Source Porting** – migrate nodes to `rclcpp`, convert parameters, and fix TF.
4. **Launch/Config** – provide `.launch.py` and YAML files with declared parameters.
5. **Testing** – add `ament_add_gtest` or launch tests; ensure they run via `colcon test --packages-select <pkg>`.
6. **Docs** – update this wiki section and `docs/porting/PORTING_PROGRESS.md` with the change date.
