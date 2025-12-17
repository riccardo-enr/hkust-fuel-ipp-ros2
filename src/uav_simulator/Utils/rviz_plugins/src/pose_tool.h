#ifndef RVIZ_PLUGINS_POSE_TOOL_H
#define RVIZ_PLUGINS_POSE_TOOL_H

#include <OgreVector3.h>
#include <QCursor>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>

namespace rviz_rendering {
class Arrow;
}

namespace rviz_common {
class DisplayContext;
}

namespace rviz_plugins {

class Pose3DTool : public rviz_common::Tool {
public:
  Pose3DTool();
  virtual ~Pose3DTool();

  virtual void onInitialize() override;

  virtual void activate() override;
  virtual void deactivate() override;

  virtual int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

protected:
  virtual void onPoseSet(double x, double y, double z, double theta) = 0;

  rviz_rendering::Arrow* arrow_;
  std::vector<rviz_rendering::Arrow*> arrow_array;

  enum State { Position, Orientation, Height };
  State state_;

  Ogre::Vector3 pos_;
};
}

#endif
