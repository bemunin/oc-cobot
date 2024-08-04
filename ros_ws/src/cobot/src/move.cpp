#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>

// alias
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Plan = moveit::planning_interface::MoveGroupInterface::Plan;
using MoveItErrorCode = moveit::core::MoveItErrorCode;

struct FkJointCommand {
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
std::vector<std::string> cmdParser(int argc, char** argv){
  std::vector<std::string> output;
  
  if(argc<2){
    output.push_back("fk:standby");
    return output;
  }

  std::string cmd = argv[1];

  if(cmd == "ik"){
    // TODO: Inverse Kinematic
    output.push_back("ik");
    return output;
  } else {
    // forward kinematic, accepts state name (pick, standby)
    std::string state = argv[1];
    output.push_back("fk:"+state);

    if(argc==3){
      std::string gripper_pos_meter = argv[2];
      output.push_back(gripper_pos_meter);
    }
    return output;
  } 
}

// Forward Kinematics
void move_fk(MoveGroupInterface &mg, FkJointCommand const &target){
  
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

  // fk states
  std::vector<double> prepick_arm_joint_degs = {-3,77,2,-60,3,164,38};
  std::vector<double> standby_arm_joint_degs = {0,-45,0,-135,0,92,45};
  std::vector<double> basket1_arm_joint_degs = {122,14,17,-82,-15,145,45};
  std::vector<double> basket2_arm_joint_degs = {166,-32,6,-134,7,127,50};


  // create joint target
  // parse commandline arguments
  std::vector<std::string> parse_values = cmdParser(argc, argv);
  std::string cmd = parse_values.at(0);

  // Inverse Kinematics

  if (cmd == "ik"){
    move_ik();
  } else {
    // Forward Kinematics
    FkJointCommand goal;
    goal.arm_joint_degs = standby_arm_joint_degs;
    goal.gripper_pos_meter = 0.04;


    if(parse_values.size() == 2){
      goal.gripper_pos_meter = std::stod(parse_values.at(1));
    }

    if(cmd == "fk:prepick"){
      goal.arm_joint_degs = prepick_arm_joint_degs;
    } else if(cmd == "fk:standby"){
      goal.arm_joint_degs = standby_arm_joint_degs;
    } else if(cmd == "fk:basket1" || cmd == "fk:basket"){
      goal.arm_joint_degs = basket1_arm_joint_degs;
    } else if(cmd == "fk:basket2"){
      basket2_arm_joint_degs.at(0) = 166;
      goal.arm_joint_degs = basket2_arm_joint_degs;
    } else if(cmd == "fk:basket3"){
      basket1_arm_joint_degs.at(0) = -158;
      goal.arm_joint_degs = basket1_arm_joint_degs;
    } 
  
    move_fk(mg_arm, goal);
  }

  rclcpp::shutdown();
  return 0;
}