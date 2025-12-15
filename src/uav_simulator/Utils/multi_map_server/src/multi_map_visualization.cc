#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <pose_utils.h>

#include "multi_map_server/Map2D.h"
#include "multi_map_server/Map3D.h"
#include "multi_map_server/msg/multi_occupancy_grid.hpp"
#include "multi_map_server/msg/multi_sparse_map3_d.hpp"

using multi_map_server::msg::MultiOccupancyGrid;
using multi_map_server::msg::MultiSparseMap3D;

class MultiMapVisualizationNode : public rclcpp::Node {
public:
  MultiMapVisualizationNode() : rclcpp::Node("multi_map_visualization") {
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");

    auto transient_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

    maps2d_pub_ = this->create_publisher<MultiOccupancyGrid>("~/maps2d", transient_qos);
    map3d_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud>("~/map3d", transient_qos);

    maps2d_sub_ = this->create_subscription<MultiOccupancyGrid>(
        "~/dmaps2d", rclcpp::QoS(1),
        std::bind(&MultiMapVisualizationNode::Maps2dCallback, this, std::placeholders::_1));
    maps3d_sub_ = this->create_subscription<MultiSparseMap3D>(
        "~/dmaps3d", rclcpp::QoS(1),
        std::bind(&MultiMapVisualizationNode::Maps3dCallback, this, std::placeholders::_1));
  }

private:
  void Maps2dCallback(const MultiOccupancyGrid::SharedPtr msg) {
    maps2d_.resize(msg->maps.size(), Map2D(4));
    for (size_t k = 0; k < msg->maps.size(); ++k) {
      maps2d_[k].Replace(msg->maps[k]);
    }
    origins2d_ = msg->origins;

    MultiOccupancyGrid merged;
    merged.maps.resize(maps2d_.size());
    merged.origins = origins2d_;
    const builtin_interfaces::msg::Time stamp = this->now();
    for (size_t k = 0; k < maps2d_.size(); ++k) {
      merged.maps[k] = maps2d_[k].GetMap(stamp, frame_id_);
    }
    maps2d_pub_->publish(merged);
  }

  void Maps3dCallback(const MultiSparseMap3D::SharedPtr msg) {
    maps3d_.resize(msg->maps.size());
    for (size_t k = 0; k < msg->maps.size(); ++k) {
      maps3d_[k].UnpackMsg(msg->maps[k]);
    }
    origins3d_ = msg->origins;

    sensor_msgs::msg::PointCloud cloud;
    for (size_t k = 0; k < msg->maps.size(); ++k) {
      arma::colvec po(6);
      po(0) = origins3d_[k].position.x;
      po(1) = origins3d_[k].position.y;
      po(2) = origins3d_[k].position.z;
      arma::colvec poq(4);
      poq(0) = origins3d_[k].orientation.w;
      poq(1) = origins3d_[k].orientation.x;
      poq(2) = origins3d_[k].orientation.y;
      poq(3) = origins3d_[k].orientation.z;
      po.rows(3, 5) = R_to_ypr(quaternion_to_R(poq));
      arma::colvec tpo = po.rows(0, 2);
      arma::mat Rpo = ypr_to_R(po.rows(3, 5));
      std::vector<arma::colvec> pts = maps3d_[k].GetOccupancyWorldFrame(OCCUPIED);
      for (size_t i = 0; i < pts.size(); ++i) {
        arma::colvec pt = Rpo * pts[i] + tpo;
        geometry_msgs::msg::Point32 point;
        point.x = pt(0);
        point.y = pt(1);
        point.z = pt(2);
        cloud.points.push_back(point);
      }
    }

    cloud.header.stamp = this->now();
    cloud.header.frame_id = frame_id_;
    map3d_pub_->publish(cloud);
  }

  std::string frame_id_;
  std::vector<Map2D> maps2d_;
  std::vector<geometry_msgs::msg::Pose> origins2d_;
  std::vector<Map3D> maps3d_;
  std::vector<geometry_msgs::msg::Pose> origins3d_;

  rclcpp::Subscription<MultiOccupancyGrid>::SharedPtr maps2d_sub_;
  rclcpp::Subscription<MultiSparseMap3D>::SharedPtr maps3d_sub_;
  rclcpp::Publisher<MultiOccupancyGrid>::SharedPtr maps2d_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr map3d_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiMapVisualizationNode>());
  rclcpp::shutdown();
  return 0;
}
