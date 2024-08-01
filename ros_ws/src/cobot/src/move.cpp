#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>

using moveit::planning_interface::MoveGroupInterface;

double degreeToRadian(double deg){
  return deg * M_PI / 180;
}

auto setJointDegrees(MoveGroupInterface &movegroup, std::vector<double> joint_goal_degs){ 
  std::vector<double> joint_goal_rads(joint_goal_degs.size());
  for(size_t i =0; i< joint_goal_degs.size(); i++){
    joint_goal_rads[i] = degreeToRadian(joint_goal_degs[i]);
  }
  movegroup.setJointValueTarget(joint_goal_rads);
}

auto setGripper(MoveGroupInterface &movegroup, double pos_meter){
  std::vector<double> finger_pos_meter{pos_meter,pos_meter};
  movegroup.setJointValueTarget(finger_pos_meter);
}

void execute(MoveGroupInterface &movegroup, rclcpp::Logger const &logger){
  auto const [success, plan] = [&movegroup]{
    MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(movegroup.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if(success) {
    movegroup.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("move");
  auto const logger = rclcpp::get_logger("cobot_move");
  RCLCPP_INFO(logger, "Start node");

  auto mg_arm = MoveGroupInterface(node, "panda_arm");

  // double gripper_pos = 0.02;
  // if(argc>1){
  //   gripper_pos = std::stod(argv[1]);
  // }

  // FK goal command
  setJointDegrees(mg_arm,{-3,77,2,-60,3,164,38});
  execute(mg_arm, logger);

  rclcpp::shutdown();
  return 0;
}