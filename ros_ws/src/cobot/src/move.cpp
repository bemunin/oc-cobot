#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("move");
  RCLCPP_INFO(rclcpp::get_logger("move"), "Start node");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}