# API Changes Reference

Summarize common code changes for quick lookup.

| Concept | ROS 1 | ROS 2 |
|---------|-------|-------|
| Node creation | `ros::NodeHandle nh;` | `class FooNode : public rclcpp::Node { ... }` |
| Logging | `ROS_INFO_STREAM` | `RCLCPP_INFO(get_logger(), ...)` |
| Parameters | `nh.param("foo", value, default);` | `value = declare_parameter("foo", default);` |
| Timers | `ros::Rate` loop | `create_wall_timer(period, callback)` |
| TF | `tf::TransformListener` | `tf2_ros::Buffer` + `tf2_ros::TransformListener` |
| Messages | `pkg/Msg.h` | `pkg/msg/msg.hpp` |
| Launch | `.launch` XML | `.launch.py` Python API |

Extend the table with QoS, QoS overrides, Composition, Lifecycle, etc., as needed.
