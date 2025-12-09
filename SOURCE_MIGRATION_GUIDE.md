# ROS 1 to ROS 2 Source Code Migration Guide

## Overview

This guide provides patterns and examples for migrating C++ source code from ROS 1 to ROS 2.

## General Migration Patterns

### 1. Header Includes

| ROS 1 | ROS 2 |
|-------|-------|
| `#include <ros/ros.h>` | `#include <rclcpp/rclcpp.hpp>` |
| `#include <nav_msgs/Odometry.h>` | `#include <nav_msgs/msg/odometry.hpp>` |
| `#include <sensor_msgs/PointCloud2.h>` | `#include <sensor_msgs/msg/point_cloud2.hpp>` |
| `#include <geometry_msgs/PoseStamped.h>` | `#include <geometry_msgs/msg/pose_stamped.hpp>` |
| `#include <tf/transform_datatypes.h>` | `#include <tf2/utils.h>` or `#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>` |
| `#include <tf/transform_broadcaster.h>` | `#include <tf2_ros/transform_broadcaster.h>` |

**Rule:** Message headers use `/msg/` subdirectory and `.hpp` extension (lowercase with underscores).

### 2. Node Structure

#### ROS 1 Pattern (Global State)
```cpp
ros::Publisher pub;
ros::Subscriber sub;
double param_value;

void callback(const std_msgs::String::ConstPtr& msg) {
    // Process message
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_name");
    ros::NodeHandle nh;
    
    nh.param("param", param_value, 0.0);
    sub = nh.subscribe("topic", 10, callback);
    pub = nh.advertise<std_msgs::String>("output", 10);
    
    ros::spin();
    return 0;
}
```

#### ROS 2 Pattern (Class-Based)
```cpp
class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("node_name") {
        // Declare and get parameters
        this->declare_parameter("param", 0.0);
        param_value_ = this->get_parameter("param").as_double();
        
        // Create subscriber and publisher
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, 
            std::bind(&MyNode::callback, this, std::placeholders::_1));
        
        pub_ = this->create_publisher<std_msgs::msg::String>("output", 10);
    }

private:
    void callback(const std_msgs::msg::String::SharedPtr msg) {
        // Process message
    }
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    double param_value_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### 3. Publishers and Subscribers

#### ROS 1
```cpp
ros::Publisher pub = nh.advertise<std_msgs::String>("topic", 10);
ros::Subscriber sub = nh.subscribe("topic", 10, callback);

// Publishing
std_msgs::String msg;
msg.data = "hello";
pub.publish(msg);
```

#### ROS 2
```cpp
auto pub = this->create_publisher<std_msgs::msg::String>("topic", 10);
auto sub = this->create_subscription<std_msgs::msg::String>(
    "topic", 10, std::bind(&Class::callback, this, std::placeholders::_1));

// Publishing
std_msgs::msg::String msg;
msg.data = "hello";
pub->publish(msg);
```

**Key Differences:**
- Use `SharedPtr` (pointer) to publish
- Callbacks receive `::SharedPtr` instead of `::ConstPtr`
- Use `std::bind` for member function callbacks

### 4. Parameters

#### ROS 1
```cpp
double value;
nh.param("param_name", value, 0.0);  // Gets with default
nh.getParam("param_name", value);     // Gets, returns bool
```

#### ROS 2
```cpp
// Declare first (usually in constructor)
this->declare_parameter("param_name", 0.0);

// Then get
double value = this->get_parameter("param_name").as_double();

// Or combined
this->declare_parameter("param_name", 0.0);
value = this->get_parameter("param_name").as_double();
```

### 5. Time

#### ROS 1
```cpp
ros::Time now = ros::Time::now();
ros::Duration d(1.0);
ros::Rate rate(10);  // 10 Hz

// In loop
rate.sleep();
```

#### ROS 2
```cpp
rclcpp::Time now = this->now();  // or node->now()
rclcpp::Duration d(1, 0);  // seconds, nanoseconds
// Or: rclcpp::Duration d = rclcpp::Duration::from_seconds(1.0);

// Instead of rate.sleep() in loop, use timer:
timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),  // 10 Hz
    std::bind(&MyNode::callback, this));
```

### 6. Logging

#### ROS 1
```cpp
ROS_INFO("Message: %s", str.c_str());
ROS_WARN("Warning");
ROS_ERROR("Error");
ROS_DEBUG("Debug");
```

#### ROS 2
```cpp
RCLCPP_INFO(this->get_logger(), "Message: %s", str.c_str());
RCLCPP_WARN(this->get_logger(), "Warning");
RCLCPP_ERROR(this->get_logger(), "Error");
RCLCPP_DEBUG(this->get_logger(), "Debug");
```

### 7. Timers (Replacing Rate Loops)

#### ROS 1
```cpp
ros::Rate rate(10);
while(ros::ok()) {
    doWork();
    ros::spinOnce();
    rate.sleep();
}
```

#### ROS 2
```cpp
// In class constructor:
timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MyNode::doWork, this));

// Then just spin:
rclcpp::spin(node);
```

### 8. Spinning

#### ROS 1
```cpp
ros::spin();        // Blocks until shutdown
ros::spinOnce();    // Process one batch of callbacks
```

#### ROS 2
```cpp
rclcpp::spin(node);          // Blocks until shutdown
rclcpp::spin_some(node);     // Process some callbacks, non-blocking
```

### 9. TF/TF2 Migration

#### ROS 1 (tf)
```cpp
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

tf::TransformBroadcaster br;
tf::Transform transform;
transform.setOrigin(tf::Vector3(x, y, z));
tf::Quaternion q;
q.setRPY(roll, pitch, yaw);
transform.setRotation(q);
br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base"));

double yaw = tf::getYaw(quaternion);
```

#### ROS 2 (tf2)
```cpp
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
// In constructor:
tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

geometry_msgs::msg::TransformStamped t;
t.header.stamp = this->now();
t.header.frame_id = "world";
t.child_frame_id = "base";
t.transform.translation.x = x;
t.transform.translation.y = y;
t.transform.translation.z = z;

tf2::Quaternion q;
q.setRPY(roll, pitch, yaw);
t.transform.rotation = tf2::toMsg(q);

tf_broadcaster_->sendTransform(t);

// For getting yaw from quaternion:
double yaw = tf2::getYaw(quaternion);
```

## Migration Checklist

For each source file:

- [ ] Update include statements (add `/msg/`, change `.h` to `.hpp`)
- [ ] Convert global state to class members
- [ ] Inherit from `rclcpp::Node`
- [ ] Move initialization to constructor
- [ ] Update parameter handling (declare + get pattern)
- [ ] Update publishers (use `SharedPtr`, pointer to publish)
- [ ] Update subscribers (use `std::bind`, `SharedPtr` in callback)
- [ ] Replace `ros::Time::now()` with `this->now()`
- [ ] Replace `ROS_*` logging with `RCLCPP_*`
- [ ] Replace rate loops with timers
- [ ] Update `main()` to use `rclcpp::init/spin/shutdown`
- [ ] Update TF to TF2 if needed
- [ ] Update message types (add `::msg::`)

## Common Pitfalls

1. **Forgetting `/msg/` in includes**: `nav_msgs/Odometry.h` → `nav_msgs/msg/odometry.hpp`
2. **Not using pointer for publishing**: `pub.publish(msg)` → `pub->publish(msg)`
3. **Wrong callback signature**: `::ConstPtr` → `::SharedPtr`
4. **Not declaring parameters**: Must call `declare_parameter()` before `get_parameter()`
5. **Using `ros::ok()` instead of `rclcpp::ok()`**
6. **Forgetting `this->` for node methods**: `now()` → `this->now()`

## Examples from This Migration

See these migrated files for reference:
- `uav_simulator/poscmd_2_odom/src/poscmd_2_odom.cpp` - Parameters, timers, callbacks
- `uav_simulator/map_generator/src/map_recorder.cpp` - Conditional spin
- `uav_simulator/map_generator/src/map_publisher.cpp` - Timer-based publishing

## Resources

- [ROS 2 Migration Guide](https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1.html)
- [rclcpp API Documentation](https://docs.ros.org/en/humble/p/rclcpp/)
- [TF2 Migration](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
