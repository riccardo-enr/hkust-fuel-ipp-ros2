#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

using namespace std;

class MapRecorder : public rclcpp::Node
{
public:
    MapRecorder(const string& file_path) 
        : Node("map_recorder"), file_path_(file_path), finish_(false)
    {
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/map_generator/click_map", 10,
            std::bind(&MapRecorder::cloudCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Map recorder initialized, waiting for point cloud...");
    }

    bool isFinished() const { return finish_; }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        pcl::io::savePCDFileASCII(file_path_ + string("tmp.pcd"), cloud);

        cout << "map saved." << endl;
        finish_ = true;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    string file_path_;
    bool finish_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (argc <= 1) {
        std::cout << "File path not specified" << std::endl;
        return 0;
    }

    string file_path = argv[1];
    
    auto node = std::make_shared<MapRecorder>(file_path);
    
    // Sleep briefly to allow initialization
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Spin until finished
    while (rclcpp::ok() && !node->isFinished()) {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_WARN(node->get_logger(), "[Map Recorder]: finish record map.");
    rclcpp::shutdown();
    return 0;
}
