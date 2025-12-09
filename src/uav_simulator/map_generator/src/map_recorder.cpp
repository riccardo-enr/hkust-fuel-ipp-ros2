#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <thread>
#include <chrono>

using namespace std;

bool finish = false;
string file_path;

void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  pcl::io::savePCDFileASCII(file_path + std::string("tmp.pcd"), cloud);

  cout << "map saved." << endl;
  finish = true;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("map_recorder");

  if (argc <= 1) {
    std::cout << "File path not specified" << std::endl;
    return 0;
  }

  file_path = argv[1];

  auto cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/map_generator/click_map", 10, cloudCallback);
  
  std::this_thread::sleep_for(std::chrono::seconds(1));

  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    rate.sleep();
    if (finish) break;
  }

  RCLCPP_WARN(node->get_logger(), "[Map Recorder]: finish record map.");
  rclcpp::shutdown();
  return 0;
}