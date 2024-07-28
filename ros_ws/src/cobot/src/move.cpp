#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>

using moveit::planning_interface::MoveGroupInterface;


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("move");
  auto const logger = rclcpp::get_logger("move");

  RCLCPP_INFO(logger, "Start node");

  auto mg_arm = MoveGroupInterface(node, "panda_arm");
  auto const target_pose = []{
  geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();

  mg_arm.setPoseTarget(target_pose);

  auto const [success, plan] = [&mg_arm]{
    MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(mg_arm.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if(success) {
    mg_arm.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}