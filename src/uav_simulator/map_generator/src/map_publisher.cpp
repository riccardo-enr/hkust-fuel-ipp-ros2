#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>
#include <chrono>

using namespace std;
string file_name;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("map_publisher");

  auto cloud_pub =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_generator/global_cloud", 10);
  
  if(argc <= 1) {
      std::cout << "No file path provided" << std::endl;
      return 1;
  }
  file_name = argv[1];

  std::this_thread::sleep_for(std::chrono::seconds(1));

  /* load cloud from pcd */
  pcl::PointCloud<pcl::PointXYZ> cloud;
  int status = pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloud);
  if (status == -1) {
    cout << "can't read file." << endl;
    return -1;
  }

  // Find range of map
  Eigen::Vector2d mmin(0, 0), mmax(0, 0);
  for (auto pt : cloud) {
    mmin[0] = min(mmin[0], double(pt.x));
    mmin[1] = min(mmin[1], double(pt.y));
    mmax[0] = max(mmax[0], double(pt.x));
    mmax[1] = max(mmax[1], double(pt.y));
  }

  // Add ground
  for (double x = mmin[0]; x <= mmax[0]; x += 0.1)
    for (double y = mmin[1]; y <= mmax[1]; y += 0.1) {
      cloud.push_back(pcl::PointXYZ(x, y, 0));
    }

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = "world";

  int count = 0;
  rclcpp::Rate rate(3.33); // 0.3s
  while (rclcpp::ok()) {
    rate.sleep();
    cloud_pub->publish(msg);
    ++count;
    if (count > 10) {
      // break;
    }
  }

  cout << "finish publish map." << endl;
  rclcpp::shutdown();
  return 0;
}
