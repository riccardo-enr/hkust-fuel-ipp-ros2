#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;

class MapPublisher : public rclcpp::Node
{
public:
    MapPublisher(const string& file_name) 
        : Node("map_publisher"), file_name_(file_name)
    {
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/map_generator/global_cloud", 10);
        
        // Load cloud from pcd
        if (!loadMap()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map from file: %s", file_name.c_str());
            return;
        }

        // Create timer for publishing at ~3Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(300),
            std::bind(&MapPublisher::publishMap, this));
        
        RCLCPP_INFO(this->get_logger(), "Map publisher initialized, publishing map...");
    }

private:
    bool loadMap()
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        int status = pcl::io::loadPCDFile<pcl::PointXYZ>(file_name_, cloud);
        if (status == -1) {
            cout << "can't read file." << endl;
            return false;
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

        // Convert to ROS message
        pcl::toROSMsg(cloud, cloud_msg_);
        cloud_msg_.header.frame_id = "world";
        
        return true;
    }

    void publishMap()
    {
        cloud_msg_.header.stamp = this->now();
        cloud_pub_->publish(cloud_msg_);
        publish_count_++;
        
        if (publish_count_ > 10) {
            // Could stop publishing here if desired
            // timer_->cancel();
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::PointCloud2 cloud_msg_;
    string file_name_;
    int publish_count_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    if (argc <= 1) {
        std::cout << "File name not specified" << std::endl;
        return -1;
    }

    string file_name = argv[1];
    
    auto node = std::make_shared<MapPublisher>(file_name);
    rclcpp::spin(node);
    
    cout << "finish publish map." << endl;
    rclcpp::shutdown();
    return 0;
}