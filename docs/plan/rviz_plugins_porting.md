# Porting Plan: `rviz_plugins`

## Overview
The `rviz_plugins` package provides custom RViz displays for the UAV simulator, specifically `MultiProbMapDisplay` (and potentially others). It currently relies on ROS 1 `roscpp`, `rviz`, and `ogre`.

## Dependency Status
| Dependency | ROS 1 | ROS 2 Equivalent | Status |
| :--- | :--- | :--- | :--- |
| Build System | `catkin` | `ament_cmake` | Needs migration |
| Core | `roscpp` | `rclcpp` | Needs migration |
| Visualization | `rviz` | `rviz_common`, `rviz_default_plugins`, `rviz_rendering`, `rviz_ogre_vendor` | Needs migration |
| Messages | `multi_map_server` | `multi_map_server` | **Ported** |
| GUI | `Qt5` | `Qt5` (via `rviz_common`) | Needs configuration |
| Graphics | `Ogre` | `Ogre` (via `rviz_ogre_vendor`) | Needs verification |

## Migration Steps

### 1. `package.xml` Update
- **Format:** Ensure `format="3"`.
- **Build Tool:** Change `catkin` to `ament_cmake`.
- **Dependencies:**
    - Remove `roscpp`, `rviz`.
    - Add `rclcpp`, `rviz_common`, `rviz_default_plugins`, `rviz_rendering`, `rviz_ogre_vendor`.
    - Keep `multi_map_server` (ensure it's an `exec_depend` or `depend`).
    - Add `qt5` dependencies if explicitly needed, though `rviz_common` usually handles exports.

### 2. `CMakeLists.txt` Rewrite
- **Initialization:** `cmake_minimum_required(VERSION 3.8)`, `project(rviz_plugins)`.
- **Find Packages:**
    - `find_package(ament_cmake REQUIRED)`
    - `find_package(rclcpp REQUIRED)`
    - `find_package(rviz_common REQUIRED)`
    - `find_package(rviz_rendering REQUIRED)`
    - `find_package(multi_map_server REQUIRED)`
    - `find_package(Qt5 COMPONENTS Widgets REQUIRED)` (usually needed for plugins).
- **MOC:** `set(CMAKE_AUTOMOC ON)` to handle Qt signals/slots automatically.
- **Library Target:**
    - `add_library(${PROJECT_NAME} src/multi_probmap_display.cpp ...)`
    - `ament_target_dependencies(${PROJECT_NAME} rclcpp rviz_common rviz_rendering multi_map_server)`
    - `target_link_libraries(${PROJECT_NAME} Qt5::Widgets)`
- **Plugin Export:**
    - Ensure `plugin_description.xml` is exported via `pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)`.
- **Install Rules:**
    - Install the library to `lib/${PROJECT_NAME}` (or global lib if preferred, but usually plugin libs go to `lib` or `lib/${PROJECT_NAME}`).
    - Install `plugin_description.xml` to `share/${PROJECT_NAME}`.
    - Install icons/media if any.

### 3. Source Code Porting
- **Headers:**
    - `#include <ros/ros.h>` -> `#include <rclcpp/rclcpp.hpp>`
    - `#include <rviz/display.h>` -> `#include <rviz_common/display.hpp>` (or `rviz_common/ros_topic_display.hpp`)
- **Class Definition:**
    - Inherit from `rviz_common::Display` (or `rviz_common::RosTopicDisplay<MsgType>` if it's a simple subscriber).
    - If inheriting from `Display`:
        - `onInitialize()`: Use `rviz_common::DisplayContext`.
        - `fixed_frame_`: Access via `context_->getFrameManager()`.
- **Node Access:**
    - Use `rclcpp::Node::SharedPtr node = context_->getRosNodeAbstraction().lock()->getRawNode();` (common pattern in porting, though `RosTopicDisplay` handles subscriptions for you).
- **Update Loop:**
    - `virtual void update(float wall_dt, float ros_dt)` is **removed** in ROS 2.
    - Logic must be moved to message callbacks or a separate timer if animation is strictly time-based.
    - For Ogre updates, usually done in `processMessage` or by queuing a render.
- **Pluginlib:**
    - `#include <pluginlib/class_list_macros.h>` -> `#include <pluginlib/class_list_macros.hpp>`
    - `PLUGINLIB_EXPORT_CLASS(..., ...)`: Update namespace/types.

### 4. Plugin Description (`plugin_description.xml`)
- Update `base_class_type` to `rviz_common::Display`.
- Update `type` to the new class namespace if changed.

## Compilation & Testing
1. `colcon build --packages-select rviz_plugins`
2. Source workspace.
3. Run `rviz2` and try to add the plugin.

## Notes
- `multi_map_server` must be successfully built first.
- If the plugin uses raw Ogre, ensure headers are found (often `rviz_rendering` exposes them).
