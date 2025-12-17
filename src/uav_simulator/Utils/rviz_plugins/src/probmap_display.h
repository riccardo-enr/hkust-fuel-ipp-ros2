#ifndef RVIZ_PLUGINS_PROB_MAP_DISPLAY_H
#define RVIZ_PLUGINS_PROB_MAP_DISPLAY_H

#include <OgreTexture.h>
#include <OgreMaterial.h>
#include <OgreVector3.h>
#include <OgreManualObject.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rviz_common/display.hpp>

namespace rviz_common {
namespace properties {
class FloatProperty;
class IntProperty;
class Property;
class QuaternionProperty;
class RosTopicProperty;
class VectorProperty;
}
}

namespace rviz_plugins {

class ProbMapDisplay : public rviz_common::Display {
  Q_OBJECT
public:
  ProbMapDisplay();
  virtual ~ProbMapDisplay();

  // Overrides from Display
  virtual void onInitialize() override;
  virtual void fixedFrameChanged() override;
  virtual void reset() override;
  virtual void update(float wall_dt, float ros_dt) override;

  float getResolution() {
    return resolution_;
  }
  int getWidth() {
    return width_;
  }
  int getHeight() {
    return height_;
  }
  Ogre::Vector3 getPosition() {
    return position_;
  }
  Ogre::Quaternion getOrientation() {
    return orientation_;
  }

protected Q_SLOTS:
  void updateAlpha();
  void updateTopic();
  void updateDrawUnder();

protected:
  // overrides from Display
  virtual void onEnable() override;
  virtual void onDisable() override;

  virtual void subscribe();
  virtual void unsubscribe();

  void incomingMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

  void clear();

  void transformMap();

  Ogre::ManualObject* manual_object_;
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;
  bool loaded_;

  std::string topic_;
  float resolution_;
  int width_;
  int height_;
  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;
  std::string frame_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Node::SharedPtr node_;

  rviz_common::properties::RosTopicProperty* topic_property_;
  rviz_common::properties::FloatProperty* resolution_property_;
  rviz_common::properties::IntProperty* width_property_;
  rviz_common::properties::IntProperty* height_property_;
  rviz_common::properties::VectorProperty* position_property_;
  rviz_common::properties::QuaternionProperty* orientation_property_;
  rviz_common::properties::FloatProperty* alpha_property_;
  rviz_common::properties::Property* draw_under_property_;

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr updated_map_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr current_map_;
  std::mutex mutex_;
  bool new_map_;
};

}  // namespace rviz_plugins

#endif