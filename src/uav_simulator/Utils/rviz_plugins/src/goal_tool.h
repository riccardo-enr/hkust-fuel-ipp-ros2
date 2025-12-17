#ifndef RVIZ_PLUGINS_GOAL_TOOL_H
#define RVIZ_PLUGINS_GOAL_TOOL_H

#ifndef Q_MOC_RUN
#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "pose_tool.h"
#endif

namespace rviz_common {
class DisplayContext;
namespace properties {
class StringProperty;
}
}

namespace rviz_plugins {

class Goal3DTool : public Pose3DTool {
  Q_OBJECT
public:
  Goal3DTool();
  virtual ~Goal3DTool() {}
  virtual void onInitialize() override;

protected:
  virtual void onPoseSet(double x, double y, double z, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;

  rviz_common::properties::StringProperty* topic_property_;
};
}

#endif