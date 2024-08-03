#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>

// alias
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Plan = moveit::planning_interface::MoveGroupInterface::Plan;
using MoveItErrorCode = moveit::core::MoveItErrorCode;

struct JointCommand {
  std::vector<double> arm_joint_degs;
  double gripper_pos_meter;
};

// global vars
rclcpp::Logger const logger = rclcpp::get_logger("cobot_move");
const int PANDA_JOINTS_NUM = 9;

// utils
double degreeToRadian(double deg){
  return deg * M_PI / 180.0;
}

// helpers
std::string cmdParser(int argc, char** argv){
  if(argc<2){
    return "fk:pick";
  }

  std::string cmd = argv[1];
  std::string output = "";

  if(cmd == "ik"){
    // TODO: Inverse Kinematic
    return "ik:0,0,0";
  } else {
    // forward kinematic, accepts state name (pick, standby)
    output.append("fk:");
    output.append(argv[1]);
    return output;
  } 
}

// Forward Kinematics
void move_fk(MoveGroupInterface &mg, JointCommand const &target){
  
  std::vector<double> joints_goal;
  joints_goal.reserve(PANDA_JOINTS_NUM);

  // push arm joint value in radian
  for(size_t i =0; i< target.arm_joint_degs.size(); i++){
    joints_goal.push_back(degreeToRadian(target.arm_joint_degs[i]));
  }

  // push gripper 2 joint values, finger_left and right 
  joints_goal.push_back(target.gripper_pos_meter);
  joints_goal.push_back(target.gripper_pos_meter);

  bool is_in_boundary = mg.setJointValueTarget(joints_goal);
  if(!is_in_boundary){
    RCLCPP_WARN(logger, "Target joints are outside of the boundary!!");
    return;
  }

  Plan plan;
  bool is_success = mg.plan(plan) == MoveItErrorCode::SUCCESS;

  if(is_success) {
    mg.move();
  } else {
    RCLCPP_ERROR(logger, "Forward kinematics planing failed!");
  }
}

// Inverse Kinematics
void move_ik(){
  // TODO
}



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("move");
  RCLCPP_INFO(logger, "Start node");
  auto mg_arm = MoveGroupInterface(node, "panda_arm_hand");

  // parse command
  std::string cmd = cmdParser(argc, argv);

  // states
  JointCommand pick_state;
  pick_state.arm_joint_degs = {-3,77,2,-60,3,164,38};
  pick_state.gripper_pos_meter = 0.04;

  JointCommand stand_by_state;
  stand_by_state.arm_joint_degs = {0,-45,0,-135,0,92,45};
  stand_by_state.gripper_pos_meter = 0.04;

  // execute
  if(cmd == "fk:pick") move_fk(mg_arm, pick_state);
  else if(cmd == "fk:standby") move_fk(mg_arm, stand_by_state);
  else move_ik();

  rclcpp::shutdown();
  return 0;
}