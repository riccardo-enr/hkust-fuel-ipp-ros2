#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgreHardwarePixelBuffer.h>

#include <tf2/utils.h>
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

#include "multi_probmap_display.h"

namespace rviz_plugins {

MultiProbMapDisplay::MultiProbMapDisplay() : rviz_common::Display(), loaded_(false), new_map_(false) {
  topic_property_ = new rviz_common::properties::RosTopicProperty(
      "Topic", "",
      QString::fromStdString("multi_map_server/msg/MultiOccupancyGrid"),
      "multi_map_server::msg::MultiOccupancyGrid topic to subscribe to.", this, SLOT(updateTopic()));

  draw_under_property_ =
      new rviz_common::properties::Property("Draw Behind", false, "Rendering option, controls whether or not the map is always"
                                         " drawn behind everything else.",
                   this, SLOT(updateDrawUnder()));
}

MultiProbMapDisplay::~MultiProbMapDisplay() {
  unsubscribe();
  clear();
}

void MultiProbMapDisplay::onInitialize() {
    node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    updateTopic();
}

void MultiProbMapDisplay::onEnable() {
  subscribe();
}

void MultiProbMapDisplay::onDisable() {
  unsubscribe();
  clear();
}

void MultiProbMapDisplay::subscribe() {
  if (!isEnabled()) {
    return;
  }

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
    
    map_sub_ = node_->create_subscription<multi_map_server::msg::MultiOccupancyGrid>(
      topic_, qos,
      std::bind(&MultiProbMapDisplay::incomingMap, this, std::placeholders::_1));
    
    setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
  } catch (const std::exception& e) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
  }
}

void MultiProbMapDisplay::unsubscribe() {
  map_sub_.reset();
  topic_ = "";
}

void MultiProbMapDisplay::updateDrawUnder() {
  bool draw_under = draw_under_property_->getValue().toBool();

  for (unsigned int k = 0; k < material_.size(); k++)
    material_[k]->setDepthWriteEnabled(!draw_under);

  for (unsigned int k = 0; k < manual_object_.size(); k++) {
    if (draw_under)
      manual_object_[k]->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
    else
      manual_object_[k]->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
  }
}

void MultiProbMapDisplay::updateTopic() {
  unsubscribe();
  subscribe();
  clear();
}

void MultiProbMapDisplay::clear() {
  setStatus(rviz_common::properties::StatusProperty::Warn, "Message", "No map received");

  if (!loaded_) {
    return;
  }

  for (unsigned k = 0; k < manual_object_.size(); k++) {
    if (manual_object_[k])
        context_->getSceneManager()->destroyManualObject(manual_object_[k]);
    
    if (!texture_[k].isNull()) {
        std::string tex_name = texture_[k]->getName();
        texture_[k].setNull();
        Ogre::TextureManager::getSingleton().unload(tex_name);
    }
  }
  manual_object_.clear();
  texture_.clear();
  material_.clear();

  loaded_ = false;
}

void MultiProbMapDisplay::incomingMap(const multi_map_server::msg::MultiOccupancyGrid::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  updated_map_ = msg;
  new_map_ = true;
}

void MultiProbMapDisplay::update(float wall_dt, float ros_dt) {
  (void)wall_dt;
  (void)ros_dt;
  
  multi_map_server::msg::MultiOccupancyGrid::ConstSharedPtr map;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!new_map_) return;
    map = updated_map_;
    new_map_ = false;
  }
  
  if (!map) return;
  current_map_ = map;

  clear();
  
  if (!current_map_->maps.empty()) {
      frame_ = current_map_->maps[0].header.frame_id;
      if (frame_.empty()) frame_ = "map";
      
      Ogre::Vector3 position;
      Ogre::Quaternion orientation;
      if (context_->getFrameManager()->getTransform(frame_, rclcpp::Time(0), position, orientation)) {
        scene_node_->setPosition(position);
        scene_node_->setOrientation(orientation);
      }
  }

  for (unsigned int k = 0; k < current_map_->maps.size(); k++) {
    if (current_map_->maps[k].data.empty()) continue;
    
    float resolution = current_map_->maps[k].info.resolution;
    int width = current_map_->maps[k].info.width;
    int height = current_map_->maps[k].info.height;

    unsigned int pixels_size = width * height;
    unsigned char* pixels = new unsigned char[pixels_size];
    memset(pixels, 255, pixels_size);
    unsigned int num_pixels_to_copy = pixels_size;
    if (pixels_size != current_map_->maps[k].data.size())
      if (current_map_->maps[k].data.size() < pixels_size)
        num_pixels_to_copy = current_map_->maps[k].data.size();
        
    for (unsigned int pixel_index = 0; pixel_index < num_pixels_to_copy; pixel_index++) {
      unsigned char val;
      int8_t data = current_map_->maps[k].data[pixel_index];
      if (data > 0)
        val = 255;
      else if (data < 0)
        val = 180;
      else
        val = 0;
      pixels[pixel_index] = val;
    }

    Ogre::DataStreamPtr pixel_stream;
    pixel_stream.bind(new Ogre::MemoryDataStream(pixels, pixels_size));
    static int tex_count = 0;
    std::stringstream ss1;
    ss1 << "MultiMapTexture" << tex_count++;
    Ogre::TexturePtr _texture_;

    _texture_ = Ogre::TextureManager::getSingleton().loadRawData(
        ss1.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, pixel_stream, width, height,
        Ogre::PF_L8, Ogre::TEX_TYPE_2D, 0);

    texture_.push_back(_texture_);
    delete[] pixels;

    static int material_count = 0;
    std::stringstream ss0;
    ss0 << "MultiMapObjectMaterial" << material_count++;
    Ogre::MaterialPtr _material_;
    _material_ = Ogre::MaterialManager::getSingleton().create(
        ss0.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    _material_->setReceiveShadows(false);
    _material_->getTechnique(0)->setLightingEnabled(false);
    _material_->setDepthBias(-16.0f, 0.0f);
    _material_->setCullingMode(Ogre::CULL_NONE);
    _material_->setDepthWriteEnabled(false);
    material_.push_back(_material_);
    material_.back()->setSceneBlending(Ogre::SBT_TRANSPARENT_COLOUR);
    material_.back()->setDepthWriteEnabled(false);
    
    Ogre::Pass* pass = material_.back()->getTechnique(0)->getPass(0);
    Ogre::TextureUnitState* tex_unit = NULL;
    if (pass->getNumTextureUnitStates() > 0)
      tex_unit = pass->getTextureUnitState(0);
    else
      tex_unit = pass->createTextureUnitState();

    tex_unit->setTextureName(_texture_->getName());
    tex_unit->setTextureFiltering(Ogre::TFO_NONE);

    static int map_count = 0;
    std::stringstream ss2;
    ss2 << "MultiMapObject" << map_count++;
    Ogre::ManualObject* _manual_object_ = context_->getSceneManager()->createManualObject(ss2.str());
    manual_object_.push_back(_manual_object_);
    scene_node_->attachObject(manual_object_.back());
    
    float yo = 0.0f; 
    float dxo = 0.0f;
    float dyo = 0.0f;
    
    if (k < current_map_->origins.size()) {
        yo = tf2::getYaw(current_map_->origins[k].orientation);
        dxo = current_map_->origins[k].position.x;
        dyo = current_map_->origins[k].position.y;
    }
    
    float co = cos(yo);
    float so = sin(yo);
    
    float yaw = tf2::getYaw(current_map_->maps[k].info.origin.orientation);
    float dxm = current_map_->maps[k].info.origin.position.x;
    float dym = current_map_->maps[k].info.origin.position.y;
    
    float dx = co * dxm - so * dym + dxo;
    float dy = so * dxm + co * dym + dyo;
    
    float total_yaw = yaw + yo;
    float c = cos(total_yaw);
    float s = sin(total_yaw);
    
    float x = 0.0;
    float y = 0.0;
    
    manual_object_.back()->begin(material_.back()->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
    {
      {
        x = c * 0.0 - s * 0.0 + dx;
        y = s * 0.0 + c * 0.0 + dy;
        manual_object_.back()->position(x, y, 0.0f);
        manual_object_.back()->textureCoord(0.0f, 0.0f);
        manual_object_.back()->normal(0.0f, 0.0f, 1.0f);

        x = c * resolution * width - s * resolution * height + dx;
        y = s * resolution * width + c * resolution * height + dy;
        manual_object_.back()->position(x, y, 0.0f);
        manual_object_.back()->textureCoord(1.0f, 1.0f);
        manual_object_.back()->normal(0.0f, 0.0f, 1.0f);

        x = c * 0.0 - s * resolution * height + dx;
        y = s * 0.0 + c * resolution * height + dy;
        manual_object_.back()->position(x, y, 0.0f);
        manual_object_.back()->textureCoord(0.0f, 1.0f);
        manual_object_.back()->normal(0.0f, 0.0f, 1.0f);
      }

      {
        x = c * 0.0 - s * 0.0 + dx;
        y = s * 0.0 + c * 0.0 + dy;
        manual_object_.back()->position(x, y, 0.0f);
        manual_object_.back()->textureCoord(0.0f, 0.0f);
        manual_object_.back()->normal(0.0f, 0.0f, 1.0f);

        x = c * resolution * width - s * 0.0 + dx;
        y = s * resolution * width + c * 0.0 + dy;
        manual_object_.back()->position(x, y, 0.0f);
        manual_object_.back()->textureCoord(1.0f, 0.0f);
        manual_object_.back()->normal(0.0f, 0.0f, 1.0f);

        x = c * resolution * width - s * resolution * height + dx;
        y = s * resolution * width + c * resolution * height + dy;
        manual_object_.back()->position(x, y, 0.0f);
        manual_object_.back()->textureCoord(1.0f, 1.0f);
        manual_object_.back()->normal(0.0f, 0.0f, 1.0f);
      }
    }
    manual_object_.back()->end();
    
    if (draw_under_property_->getValue().toBool())
      manual_object_.back()->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
  }
  
  setStatus(rviz_common::properties::StatusProperty::Ok, "Message", "Map received");
  loaded_ = true;
  context_->queueRender();
}

void MultiProbMapDisplay::reset() {
  rviz_common::Display::reset();
  clear();
  updateTopic();
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::MultiProbMapDisplay, rviz_common::Display)