#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <pcl/io/obj_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("load_obj");

  auto cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/load_obj/cloud", rclcpp::QoS(10));

  pcl::PointCloud<pcl::PointXYZ> cloud;

  // pcl::io::loadOBJFile("/home/boboyu/Downloads/AnyConv.com__truss_bridge.obj", cloud);
  pcl::io::loadPCDFile<pcl::PointXYZ>("/home/boboyu/Downloads/pp.pcd", cloud);

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = "world";

  // Rotate the cloud
  for (int i = 0; i < cloud.points.size(); ++i) {
    auto pt = cloud.points[i];
    pcl::PointXYZ pr;
    pr.x = pt.x;
    pr.y = -pt.z;
    pr.z = pt.y;
    cloud.points[i] = pr;
  }

  sensor_msgs::msg::PointCloud2 cloud2;
  pcl::toROSMsg(cloud, cloud2);
  cloud2.header.frame_id = "world";

  rclcpp::WallRate rate(5.0);
  while (rclcpp::ok()) {
    cloud2.header.stamp = node->get_clock()->now();
    cloud_pub->publish(cloud2);
    rate.sleep();
  }

  std::cout << "Cloud published!" << std::endl;

  rclcpp::shutdown();
  return 0;
}
