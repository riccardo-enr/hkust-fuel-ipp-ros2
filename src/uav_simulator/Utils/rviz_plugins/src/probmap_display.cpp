#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgreHardwarePixelBuffer.h>

#include <tf2_ros/transform_listener.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_rendering/objects/grid.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/quaternion_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_common/logging.hpp>

#include "probmap_display.h"

namespace rviz_plugins {

ProbMapDisplay::ProbMapDisplay()
  : rviz_common::Display()
  , manual_object_(NULL)
  , loaded_(false)
  , resolution_(0.0f)
  , width_(0)
  , height_(0)
  , position_(Ogre::Vector3::ZERO)
  , orientation_(Ogre::Quaternion::IDENTITY)
  , new_map_(false) {
  
  topic_property_ = new rviz_common::properties::RosTopicProperty(
      "Topic", "", QString::fromStdString("nav_msgs/msg/OccupancyGrid"),
      "nav_msgs::msg::OccupancyGrid topic to subscribe to.", this, SLOT(updateTopic()));

  alpha_property_ = new rviz_common::properties::FloatProperty("Alpha", 0.7, "Amount of transparency to apply to the map.", this,
                                      SLOT(updateAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  draw_under_property_ =
      new rviz_common::properties::Property("Draw Behind", false, "Rendering option, controls whether or not the map is always"
                                         " drawn behind everything else.",
                   this, SLOT(updateDrawUnder()));

  resolution_property_ =
      new rviz_common::properties::FloatProperty("Resolution", 0, "Resolution of the map. (not editable)", this);
  resolution_property_->setReadOnly(true);

  width_property_ = new rviz_common::properties::IntProperty("Width", 0, "Width of the map, in meters. (not editable)", this);
  width_property_->setReadOnly(true);

  height_property_ = new rviz_common::properties::IntProperty("Height", 0, "Height of the map, in meters. (not editable)", this);
  height_property_->setReadOnly(true);

  position_property_ = new rviz_common::properties::VectorProperty(
      "Position", Ogre::Vector3::ZERO,
      "Position of the bottom left corner of the map, in meters. (not editable)", this);
  position_property_->setReadOnly(true);

  orientation_property_ = new rviz_common::properties::QuaternionProperty("Orientation", Ogre::Quaternion::IDENTITY,
                                                 "Orientation of the map. (not editable)", this);
  orientation_property_->setReadOnly(true);
}

ProbMapDisplay::~ProbMapDisplay() {
  unsubscribe();
  clear();
}

void ProbMapDisplay::onInitialize() {
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();

  static int count = 0;
  std::stringstream ss;
  ss << "ProbMapObject" << count++;
  
  manual_object_ = context_->getSceneManager()->createManualObject(ss.str());
  manual_object_->setDynamic(true);
  scene_node_->attachObject(manual_object_);

  ss << "Material";
  material_ = Ogre::MaterialManager::getSingleton().create(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material_->setCullingMode(Ogre::CULL_NONE);
  material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  material_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);

  updateAlpha();
  updateDrawUnder();
  updateTopic();
}

void ProbMapDisplay::onEnable() {
  subscribe();
}

void ProbMapDisplay::onDisable() {
  unsubscribe();
  clear();
}

void ProbMapDisplay::subscribe() {
  if (!isEnabled())
    return;

  std::string topic_name = topic_property_->getTopicStd();
  if (topic_name.empty())
    return;

  std::string resolved_name = context_->getRosNodeAbstraction().lock()->get_raw_node()->get_node_topics_interface()->resolve_topic_name(topic_name, false);
  if (resolved_name.empty()) {
     setStatus(rviz_common::properties::StatusProperty::Error, "Topic", "Invalid topic name");
     return;
  }

  if (resolved_name == topic_) {
    return;
  }
  unsubscribe();
  topic_ = resolved_name;
  
  try {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local();
    
    map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      topic_, qos,
      std::bind(&ProbMapDisplay::incomingMap, this, std::placeholders::_1));
    
    setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
  } catch (const std::exception& e) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
  }
}

void ProbMapDisplay::unsubscribe() {
  map_sub_.reset();
  topic_ = "";
}

void ProbMapDisplay::updateTopic() {
  unsubscribe();
  subscribe();
  clear();
}

void ProbMapDisplay::updateAlpha() {
  float alpha = alpha_property_->getFloat();
  if (material_) {
    material_->getTechnique(0)->getPass(0)->setDiffuse(0.0, 0.0, 0.0, alpha);
    material_->getTechnique(0)->getPass(0)->setAmbient(0.0, 0.0, 0.0);
    material_->getTechnique(0)->getPass(0)->setSelfIllumination(0.0, 0.0, 0.0);
  }
}

void ProbMapDisplay::updateDrawUnder() {
  bool draw_under = draw_under_property_->getValue().toBool();
  if (alpha_property_->getFloat() >= 0.9998) {
    if (manual_object_)
      manual_object_->setRenderQueueGroup(draw_under ? Ogre::RENDER_QUEUE_3 : Ogre::RENDER_QUEUE_MAIN);
  } else {
    if (manual_object_)
      manual_object_->setRenderQueueGroup(draw_under ? Ogre::RENDER_QUEUE_3 : Ogre::RENDER_QUEUE_MAIN);
  }
}

void ProbMapDisplay::clear() {
  setStatus(rviz_common::properties::StatusProperty::Warn, "Message", "No map received");
  if (manual_object_)
    manual_object_->clear();
  loaded_ = false;
}

void ProbMapDisplay::incomingMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  updated_map_ = msg;
  new_map_ = true;
}

void ProbMapDisplay::update(float wall_dt, float ros_dt) {
  (void)wall_dt;
  (void)ros_dt;
  
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!new_map_)
      return;
    map = updated_map_;
    new_map_ = false;
  }
  
  if (!map) return;

  current_map_ = map;
  
  if (map->info.width * map->info.height == 0) {
    return;
  }
  
  // Transform
  frame_ = map->header.frame_id;
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  
  if (!context_->getFrameManager()->getTransform(map->header.frame_id, map->header.stamp, position, orientation)) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Transform", QString("Failed to transform from frame [%1] to fixed frame [%2]").arg(QString::fromStdString(frame_)).arg(context_->getFixedFrame()));
    return;
  }
  
  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  resolution_ = map->info.resolution;
  width_ = map->info.width;
  height_ = map->info.height;
  position_ = Ogre::Vector3(map->info.origin.position.x, map->info.origin.position.y, map->info.origin.position.z);
  orientation_ = Ogre::Quaternion(map->info.origin.orientation.w, map->info.origin.orientation.x, map->info.origin.orientation.y, map->info.origin.orientation.z);
  
  resolution_property_->setValue(resolution_);
  width_property_->setValue(width_);
  height_property_->setValue(height_);
  position_property_->setVector(position_);
  orientation_property_->setQuaternion(orientation_);
  
  // Update texture
  int num_pixels = width_ * height_;
  
  std::string texture_name = "ProbMapTexture" + std::to_string(width_) + "_" + std::to_string(height_);
  if (texture_.isNull() || texture_->getName() != texture_name || texture_->getWidth() != (unsigned int)width_ || texture_->getHeight() != (unsigned int)height_) {
     if (!texture_.isNull()) {
        Ogre::TextureManager::getSingleton().remove(texture_->getHandle());
     }
     texture_ = Ogre::TextureManager::getSingleton().createManual(
        texture_name,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,
        width_, height_, 1,
        0,
        Ogre::PF_BYTE_BGRA,
        Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE
     );
  }
  
  Ogre::HardwarePixelBufferSharedPtr pixel_buffer = texture_->getBuffer();
  pixel_buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
  const Ogre::PixelBox& pixel_box = pixel_buffer->getCurrentLock();
  uint8_t* p_dest = static_cast<uint8_t*>(pixel_box.data);
  
  for (int i = 0; i < num_pixels; ++i) {
    int val = map->data[i];
    uint8_t v;
    uint8_t alpha = 255;
    if (val == -1) {
        v = 255; 
        alpha = 0;
    } else {
        v = 255 - (255 * val) / 100;
        alpha = 255;
    }
    
    *p_dest++ = v; // B
    *p_dest++ = v; // G
    *p_dest++ = v; // R
    *p_dest++ = alpha; // A
  }
  
  pixel_buffer->unlock();
  
  if (material_) {
    Ogre::TextureUnitState* tex_unit = NULL;
    if (material_->getTechnique(0)->getPass(0)->getNumTextureUnitStates() > 0)
    {
      tex_unit = material_->getTechnique(0)->getPass(0)->getTextureUnitState(0);
    }
    else
    {
      tex_unit = material_->getTechnique(0)->getPass(0)->createTextureUnitState();
    }
    tex_unit->setTextureName(texture_->getName());
    tex_unit->setTextureFiltering(Ogre::TFO_NONE);
  }
  
  // Update ManualObject
  manual_object_->clear();
  manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
  
  float w = width_ * resolution_;
  float h = height_ * resolution_;
  
  manual_object_->position(0, 0, 0);
  manual_object_->textureCoord(0, 0);
  manual_object_->normal(0, 0, 1);
  
  manual_object_->position(w, 0, 0);
  manual_object_->textureCoord(1, 0);
  manual_object_->normal(0, 0, 1);
  
  manual_object_->position(w, h, 0);
  manual_object_->textureCoord(1, 1);
  manual_object_->normal(0, 0, 1);
  
  manual_object_->position(0, h, 0);
  manual_object_->textureCoord(0, 1);
  manual_object_->normal(0, 0, 1);
  
  manual_object_->quad(0, 1, 2, 3);
  manual_object_->end();
  
  setStatus(rviz_common::properties::StatusProperty::Ok, "Message", "Map received");
  loaded_ = true;
  
  fixedFrameChanged();
}

void ProbMapDisplay::fixedFrameChanged() {
  if (loaded_) {
     Ogre::Vector3 position;
     Ogre::Quaternion orientation;
     if (context_->getFrameManager()->getTransform(frame_, rclcpp::Time(0), position, orientation)) {
       Ogre::Vector3 map_origin = position_;
       Ogre::Quaternion map_orient = orientation_;
       
       Ogre::Vector3 final_pos = position + orientation * map_origin;
       Ogre::Quaternion final_ori = orientation * map_orient;
       
       scene_node_->setPosition(final_pos);
       scene_node_->setOrientation(final_ori);
     }
  }
}

void ProbMapDisplay::reset() {
  Display::reset();
  clear();
  updateTopic();
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::ProbMapDisplay, rviz_common::Display)