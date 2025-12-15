#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <math.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Eigen>
#include <random>

using namespace std;

class ClickMapGenerator : public rclcpp::Node
{
public:
    ClickMapGenerator() : Node("click_map")
    {
        // Declare and get parameters
        this->declare_parameter("map/len2", 0.15);
        len2_ = this->get_parameter("map/len2").as_double();

        // Create publisher and subscriber
        all_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/map_generator/click_map", 1);
        
        click_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/move_base_simple/goal", 10,
            std::bind(&ClickMapGenerator::clickCallback, this, std::placeholders::_1));

        // Create timer for periodic publishing at 1Hz
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ClickMapGenerator::publishMap, this));
        
        RCLCPP_INFO(this->get_logger(), "Click map generator initialized");
    }

private:
    void clickCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;
        points_.push_back(Eigen::Vector3d(x, y, 0));
        if (points_.size() < 2) return;

        // Generate wall using two points
        Eigen::Vector3d p1 = points_[0];
        Eigen::Vector3d p2 = points_[1];
        points_.clear();

        Eigen::Vector3d dir1 = (p2 - p1).normalized();
        double len = (p2 - p1).norm();
        Eigen::Vector3d dir2;
        dir2[0] = -dir1[1];
        dir2[1] = dir1[0];

        pcl::PointXYZ pt_random;
        for (double l1 = 0.0; l1 <= len + 1e-3; l1 += 0.1) {
            Eigen::Vector3d tmp1 = p1 + l1 * dir1;
            for (double l2 = -len2_; l2 <= len2_ + 1e-3; l2 += 0.1) {
                Eigen::Vector3d tmp2 = tmp1 + l2 * dir2;
                for (double h = -0.5; h < 2.5; h += 0.1) {
                    pt_random.x = tmp2[0];
                    pt_random.y = tmp2[1];
                    pt_random.z = h;
                    map_cloud_.push_back(pt_random);
                }
            }
        }

        map_cloud_.width = map_cloud_.points.size();
        map_cloud_.height = 1;
        map_cloud_.is_dense = true;
        
        publishMap();
    }

    void publishMap()
    {
        pcl::toROSMsg(map_cloud_, map_msg_);
        map_msg_.header.frame_id = "world";
        map_msg_.header.stamp = this->now();
        all_map_pub_->publish(map_msg_);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_map_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr click_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    sensor_msgs::msg::PointCloud2 map_msg_;
    pcl::PointCloud<pcl::PointXYZ> map_cloud_;
    vector<Eigen::Vector3d> points_;
    double len2_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClickMapGenerator>();
    
    // Sleep briefly to allow initialization
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
