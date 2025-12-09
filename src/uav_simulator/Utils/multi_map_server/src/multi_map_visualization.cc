#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <pose_utils.h>
#include <multi_map_server/msg/multi_occupancy_grid.hpp>
#include <multi_map_server/msg/multi_sparse_map3d.hpp>
#include <multi_map_server/Map2D.h>
#include <multi_map_server/Map3D.h>

class MultiMapVisualization : public rclcpp::Node {
public:
  MultiMapVisualization() : Node("multi_map_visualization") {
    // Publishers
    pub_maps2d_ = create_publisher<multi_map_server::msg::MultiOccupancyGrid>("~/maps2d", 1);
    pub_map3d_ = create_publisher<sensor_msgs::msg::PointCloud>("~/map3d", 1);
    
    // Subscribers
    sub_dmaps2d_ = create_subscription<multi_map_server::msg::MultiOccupancyGrid>(
      "~/dmaps2d", 1,
      std::bind(&MultiMapVisualization::maps2d_callback, this, std::placeholders::_1));
    sub_dmaps3d_ = create_subscription<multi_map_server::msg::MultiSparseMap3D>(
      "~/dmaps3d", 1,
      std::bind(&MultiMapVisualization::maps3d_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(get_logger(), "Multi map visualization node started");
  }

private:
  void maps2d_callback(const multi_map_server::msg::MultiOccupancyGrid::SharedPtr msg) {
    // Merge map
    maps2d_.resize(msg->maps.size(), Map2D(4));
    for (unsigned int k = 0; k < msg->maps.size(); k++)
      maps2d_[k].Replace(msg->maps[k]);
    origins2d_ = msg->origins;    
    
    // Assemble and publish map
    multi_map_server::msg::MultiOccupancyGrid m;
    m.maps.resize(maps2d_.size());
    m.origins.resize(maps2d_.size());
    for (unsigned int k = 0; k < maps2d_.size(); k++) {
      m.maps[k] = maps2d_[k].GetMap();
      // Update timestamp
      m.maps[k].header.stamp = now();
      m.maps[k].info.map_load_time = now();
      m.origins[k] = origins2d_[k];
    }
    pub_maps2d_->publish(m);
  }

  void maps3d_callback(const multi_map_server::msg::MultiSparseMap3D::SharedPtr msg) {
    // Update incremental map
    maps3d_.resize(msg->maps.size());  
    for (unsigned int k = 0; k < msg->maps.size(); k++)
      maps3d_[k].UnpackMsg(msg->maps[k]);
    origins3d_ = msg->origins;
    
    // Publish
    sensor_msgs::msg::PointCloud m;
    for (unsigned int k = 0; k < msg->maps.size(); k++) {
      colvec po(6);
      po(0) = origins3d_[k].position.x;
      po(1) = origins3d_[k].position.y;
      po(2) = origins3d_[k].position.z;
      colvec poq(4);
      poq(0) = origins3d_[k].orientation.w;
      poq(1) = origins3d_[k].orientation.x;
      poq(2) = origins3d_[k].orientation.y;
      poq(3) = origins3d_[k].orientation.z;
      po.rows(3,5) = R_to_ypr(quaternion_to_R(poq));
      colvec tpo = po.rows(0,2);
      mat    Rpo = ypr_to_R(po.rows(3,5));
      vector<colvec> pts = maps3d_[k].GetOccupancyWorldFrame(OCCUPIED);
      for (unsigned int i = 0; i < pts.size(); i++) {
        colvec pt = Rpo * pts[i] + tpo;
        geometry_msgs::msg::Point32 _pt;
        _pt.x = pt(0);
        _pt.y = pt(1);
        _pt.z = pt(2);
        m.points.push_back(_pt);
      }
    }
    // Publish
    m.header.stamp = now();
    m.header.frame_id = std::string("map");
    pub_map3d_->publish(m);
  }

  rclcpp::Publisher<multi_map_server::msg::MultiOccupancyGrid>::SharedPtr pub_maps2d_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_map3d_;
  rclcpp::Subscription<multi_map_server::msg::MultiOccupancyGrid>::SharedPtr sub_dmaps2d_;
  rclcpp::Subscription<multi_map_server::msg::MultiSparseMap3D>::SharedPtr sub_dmaps3d_;
  
  std::vector<Map2D> maps2d_;
  std::vector<geometry_msgs::msg::Pose> origins2d_;
  std::vector<Map3D> maps3d_;
  std::vector<geometry_msgs::msg::Pose> origins3d_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiMapVisualization>());
  rclcpp::shutdown();
  return 0;
}
