# Debugging Recipes

- **TF mismatches:** `ros2 run tf2_tools view_frames` and inspect the generated PDF.
- **QoS compatibility:** `ros2 topic info /traj_server/command --verbose`.
- **Profiler hooks:** run nodes with `RCLCPP_LOG_LEVEL=DEBUG` and capture logs under `log/latest/`.

Extend with command snippets, screenshots, and links to issues that discuss each scenario.
