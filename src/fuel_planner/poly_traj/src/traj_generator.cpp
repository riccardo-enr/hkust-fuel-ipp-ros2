#include "poly_traj/traj_generator_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<poly_traj::TrajGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}
