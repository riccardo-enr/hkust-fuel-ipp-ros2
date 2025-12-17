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
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/quaternion_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_common/logging.hpp>

#include "aerialmap_display.h"

namespace rviz_plugins {

AerialMapDisplay::AerialMapDisplay()
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

AerialMapDisplay::~AerialMapDisplay() {
  unsubscribe();
  clear();
}

void AerialMapDisplay::onInitialize() {
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();

  static int count = 0;
  std::stringstream ss;
  ss << "AerialMapObjectMaterial" << count++;
  material_ = Ogre::MaterialManager::getSingleton().create(
      ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setDepthBias(-16.0f, 0.0f);
  material_->setCullingMode(Ogre::CULL_NONE);
  material_->setDepthWriteEnabled(false);

  updateAlpha();
  updateTopic();
}

void AerialMapDisplay::onEnable() {
  subscribe();
}

void AerialMapDisplay::onDisable() {
  unsubscribe();
  clear();
}

void AerialMapDisplay::subscribe() {
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
      std::bind(&AerialMapDisplay::incomingAerialMap, this, std::placeholders::_1));
    
    setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
  } catch (const std::exception& e) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
  }
}

void AerialMapDisplay::unsubscribe() {
  map_sub_.reset();
  topic_ = "";
}

void AerialMapDisplay::updateAlpha() {
  float alpha = alpha_property_->getFloat();

  if (material_) {
    Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
    Ogre::TextureUnitState* tex_unit = NULL;
    if (pass->getNumTextureUnitStates() > 0) {
        tex_unit = pass->getTextureUnitState(0);
    } else {
        tex_unit = pass->createTextureUnitState();
    }

    tex_unit->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, alpha);

    if (alpha < 0.9998) {
        material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        material_->setDepthWriteEnabled(false);
    } else {
        material_->setSceneBlending(Ogre::SBT_REPLACE);
        material_->setDepthWriteEnabled(!draw_under_property_->getValue().toBool());
    }
  }
}

void AerialMapDisplay::updateDrawUnder() {
  bool draw_under = draw_under_property_->getValue().toBool();

  if (alpha_property_->getFloat() >= 0.9998) {
    if (material_) material_->setDepthWriteEnabled(!draw_under);
  }

  if (manual_object_) {
    if (draw_under) {
      manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
    } else {
      manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
    }
  }
}

void AerialMapDisplay::updateTopic() {
  unsubscribe();
  subscribe();
  clear();
}

void AerialMapDisplay::clear() {
  setStatus(rviz_common::properties::StatusProperty::Warn, "Message", "No map received");

  if (!loaded_) {
    return;
  }

  if (manual_object_) {
    context_->getSceneManager()->destroyManualObject(manual_object_);
    manual_object_ = NULL;
  }

  if (!texture_.isNull()) {
    std::string tex_name = texture_->getName();
    texture_.setNull();
    Ogre::TextureManager::getSingleton().unload(tex_name);
  }

  loaded_ = false;
}

void AerialMapDisplay::incomingAerialMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  updated_map_ = msg;
  new_map_ = true;
}

void AerialMapDisplay::update(float wall_dt, float ros_dt) {
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

  if (current_map_->data.empty()) {
    return;
  }
  
  if (current_map_->info.width * current_map_->info.height == 0) {
    setStatus(rviz_common::properties::StatusProperty::Error, "AerialMap", "AerialMap is zero-sized");
    return;
  }

  clear();
  setStatus(rviz_common::properties::StatusProperty::Ok, "Message", "AerialMap received");

  float resolution = current_map_->info.resolution;
  int width = current_map_->info.width;
  int height = current_map_->info.height;
  
  Ogre::Vector3 position(current_map_->info.origin.position.x, current_map_->info.origin.position.y,
                         current_map_->info.origin.position.z);
  Ogre::Quaternion orientation(
      current_map_->info.origin.orientation.w, current_map_->info.origin.orientation.x,
      current_map_->info.origin.orientation.y, current_map_->info.origin.orientation.z);
      
  frame_ = current_map_->header.frame_id;
  if (frame_.empty()) {
    frame_ = "map";
  }

  unsigned int pixels_size = width * height * 3;
  unsigned char* pixels = new unsigned char[pixels_size];
  memset(pixels, 255, pixels_size);
  
  bool map_status_set = false;
  unsigned int num_pixels_to_copy = pixels_size;
  if (pixels_size != current_map_->data.size()) {
    if (current_map_->data.size() < pixels_size) {
      num_pixels_to_copy = current_map_->data.size();
    }
    
    // Only warn if not close (meaning not RGB)
    if (current_map_->data.size() != pixels_size) {
         std::stringstream ss;
        ss << "Data size doesn't match width*height*3: width = " << width << 
    ", height = " << height
        << ", data size = " << current_map_->data.size();
        setStatus(rviz_common::properties::StatusProperty::Error, "AerialMap", QString::fromStdString(ss.str()));
        map_status_set = true;
    }
  }
  
  for (unsigned int pixel_index = 0; pixel_index < num_pixels_to_copy; pixel_index++) {
    pixels[pixel_index] = current_map_->data[pixel_index];
  }

  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream(pixels, pixels_size));
  static int tex_count = 0;
  std::stringstream ss;
  ss << "AerialMapTexture" << tex_count++;
  
  try {
    texture_ = Ogre::TextureManager::getSingleton().loadRawData(
        ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, pixel_stream, width, height,
        Ogre::PF_R8G8B8, Ogre::TEX_TYPE_2D, 0);

    if (!map_status_set) {
      setStatus(rviz_common::properties::StatusProperty::Ok, "AerialMap", "AerialMap OK");
    }
  } catch (Ogre::RenderingAPIException&) {
    // Fallback logic
     Ogre::Image image;
    pixel_stream->seek(0);
    float fwidth = width;
    float fheight = height;
    if (width > height) {
      float aspect = fheight / fwidth;
      fwidth = 2048;
      fheight = fwidth * aspect;
    } else {
      float aspect = fwidth / fheight;
      fheight = 2048;
      fwidth = fheight * aspect;
    }

    image.loadRawData(pixel_stream, width, height, Ogre::PF_R8G8B8);
    image.resize(fwidth, fheight, Ogre::Image::FILTER_NEAREST);
    ss << "Downsampled";
    texture_ = Ogre::TextureManager::getSingleton().loadImage(
        ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
  }
  
  delete[] pixels;

  Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
  Ogre::TextureUnitState* tex_unit = NULL;
  if (pass->getNumTextureUnitStates() > 0) {
    tex_unit = pass->getTextureUnitState(0);
  } else {
    tex_unit = pass->createTextureUnitState();
  }

  tex_unit->setTextureName(texture_->getName());
  tex_unit->setTextureFiltering(Ogre::TFO_NONE);

  static int map_count = 0;
  std::stringstream ss2;
  ss2 << "AerialMapObject" << map_count++;
  manual_object_ = context_->getSceneManager()->createManualObject(ss2.str());
  scene_node_->attachObject(manual_object_);

  manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
  {
      // Bottom left
      manual_object_->position(0.0f, 0.0f, 0.0f);
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Top right
      manual_object_->position(resolution * width, resolution * height, 0.0f);
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Top left
      manual_object_->position(0.0f, resolution * height, 0.0f);
      manual_object_->textureCoord(0.0f, 1.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);
    
    
      // Bottom left
      manual_object_->position(0.0f, 0.0f, 0.0f);
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Bottom right
      manual_object_->position(resolution * width, 0.0f, 0.0f);
      manual_object_->textureCoord(1.0f, 0.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Top right
      manual_object_->position(resolution * width, resolution * height, 0.0f);
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);
  }
  manual_object_->end();

  if (draw_under_property_->getValue().toBool()) {
    manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
  }

  resolution_property_->setValue(resolution);
  width_property_->setValue(width);
  height_property_->setValue(height);
  position_property_->setVector(position);
  orientation_property_->setQuaternion(orientation);

  transformAerialMap();

  loaded_ = true;
  context_->queueRender();
}

void AerialMapDisplay::transformAerialMap() {
  if (!current_map_) return;

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  
  if (!context_->getFrameManager()->getTransform(frame_, rclcpp::Time(0), position, orientation)) {
     setStatus(rviz_common::properties::StatusProperty::Error, "Transform",
              "No transform from [" + QString::fromStdString(frame_) + "] to [" + context_->getFixedFrame() + "]");
     return;
  }
  
  Ogre::Vector3 origin_pos(current_map_->info.origin.position.x, current_map_->info.origin.position.y, current_map_->info.origin.position.z);
  Ogre::Quaternion origin_ori(current_map_->info.origin.orientation.w, current_map_->info.origin.orientation.x, current_map_->info.origin.orientation.y, current_map_->info.origin.orientation.z);
  
  Ogre::Vector3 final_pos = position + orientation * origin_pos;
  Ogre::Quaternion final_ori = orientation * origin_ori;

  scene_node_->setPosition(final_pos);
  scene_node_->setOrientation(final_ori);
  
  setStatus(rviz_common::properties::StatusProperty::Ok, "Transform", "Transform OK");
}

void AerialMapDisplay::fixedFrameChanged() {
  transformAerialMap();
}

void AerialMapDisplay::reset() {
  rviz_common::Display::reset();
  clear();
  updateTopic();
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::AerialMapDisplay, rviz_common::Display)