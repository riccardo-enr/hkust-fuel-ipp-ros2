#ifndef RVIZ_PLUGINS_MULTI_PROB_MAP_DISPLAY_H
#define RVIZ_PLUGINS_MULTI_PROB_MAP_DISPLAY_H

#include <OgreMaterial.h>
#include <OgreTexture.h>
#include <OgreVector3.h>
#include <OgreManualObject.h>

#include <rclcpp/rclcpp.hpp>
#include <multi_map_server/msg/multi_occupancy_grid.hpp>
#include <rviz_common/display.hpp>

namespace rviz_common {
namespace properties {
class Property;
class RosTopicProperty;
}
}

namespace rviz_plugins {

class MultiProbMapDisplay : public rviz_common::Display {
  Q_OBJECT
public:
  MultiProbMapDisplay();
  virtual ~MultiProbMapDisplay();

  virtual void onInitialize() override;
  virtual void reset() override;
  virtual void update(float wall_dt, float ros_dt) override;

protected Q_SLOTS:
  void updateTopic();
  void updateDrawUnder();

protected:
  virtual void onEnable() override;
  virtual void onDisable() override;

  virtual void subscribe();
  virtual void unsubscribe();

  void incomingMap(const multi_map_server::msg::MultiOccupancyGrid::ConstSharedPtr msg);

  void clear();

  std::vector<Ogre::ManualObject*> manual_object_;
  std::vector<Ogre::TexturePtr> texture_;
  std::vector<Ogre::MaterialPtr> material_;

  bool loaded_;

  std::string topic_;
  std::string frame_;

  rclcpp::Subscription<multi_map_server::msg::MultiOccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Node::SharedPtr node_;

  rviz_common::properties::RosTopicProperty* topic_property_;
  rviz_common::properties::Property* draw_under_property_;

  multi_map_server::msg::MultiOccupancyGrid::ConstSharedPtr updated_map_;
  multi_map_server::msg::MultiOccupancyGrid::ConstSharedPtr current_map_;
  std::mutex mutex_;
  bool new_map_;
};

}  // namespace rviz_plugins

#endif