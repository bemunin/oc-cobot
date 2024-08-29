#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>
#include <tf2/utils.h>

// alias
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Plan = moveit::planning_interface::MoveGroupInterface::Plan;
using MoveItErrorCode = moveit::core::MoveItErrorCode;

struct FkJointCommand
{
  std::vector<double> arm_joint_degs;
  double gripper_pos_meter;
};

// global vars
rclcpp::Logger const logger = rclcpp::get_logger("cobot_move");
const int PANDA_JOINTS_NUM = 9;

// utils
double degreeToRadian(double deg)
{
  return deg * M_PI / 180.0;
}

// helpers
std::vector<std::string> parseCmdArguments(int argc, char** argv)
{
  std::vector<std::string> output;

  if (argc < 2)
  {
    output.push_back("fk:standby");
    return output;
  }

  std::string cmd = argv[1];

  if (cmd == "ik")
  {
    output.push_back("ik");

    for (auto i = 2; i < argc; i++)
    {
      // extract 1.0 form pattern x=1.0
      std::string arg = argv[i];
      std::string key;
      std::string value;

      size_t pos = arg.find("=");

      if (pos != std::string::npos)
      {
        key = arg.substr(0, pos);
        value = arg.substr(pos + 1);
      }
      else
      {
        value = arg;
      }

      RCLCPP_INFO_STREAM(logger, "Accept value ik " << key << ": " << value);

      output.push_back(value);
    }
    // data structure in resturn string will be
    // ik, x_meter,y_meter,z_meter,(optional) roll_degree,(optional) pitch_degree,(optional)yaw_degree
    // min size = 4, max size = 7
    return output;
  }
  else
  {
    // forward kinematic, accepts state name (pick, standby)
    std::string state = argv[1];
    output.push_back("fk:" + state);

    if (argc == 3)
    {
      std::string gripper_pos_meter = argv[2];
      output.push_back(gripper_pos_meter);
    }
    return output;
  }
}

geometry_msgs::msg::Pose createGeometryMsg(std::vector<std::string> parse_values)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x = std::stod(parse_values.at(1));
  msg.position.y = std::stod(parse_values.at(2));
  msg.position.z = std::stod(parse_values.at(3));

  if (parse_values.size() == 7)
  {
    tf2::Quaternion q;
    q.setRPY(std::stod(parse_values.at(4)), std::stod(parse_values.at(5)), std::stod(parse_values.at(6)));
    msg.orientation.w = q.getW();
    msg.orientation.x = q.getX();
    msg.orientation.y = q.getY();
    msg.orientation.z = q.getZ();
  }
  else
  {
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    msg.orientation.w = q.getW();
    msg.orientation.x = q.getX();
    msg.orientation.y = q.getY();
    msg.orientation.z = q.getZ();
  }

  return msg;
}

// Forward Kinematics
void move_fk(MoveGroupInterface& mg, FkJointCommand const& target)
{
  std::vector<double> joints_goal;
  joints_goal.reserve(PANDA_JOINTS_NUM);

  // push arm joint value in radian
  for (size_t i = 0; i < target.arm_joint_degs.size(); i++)
  {
    joints_goal.push_back(degreeToRadian(target.arm_joint_degs[i]));
  }

  // push gripper 2 joint values, finger_left and right
  joints_goal.push_back(target.gripper_pos_meter);
  joints_goal.push_back(target.gripper_pos_meter);

  bool is_in_boundary = mg.setJointValueTarget(joints_goal);
  if (!is_in_boundary)
  {
    RCLCPP_WARN(logger, "Target joints are outside of the boundary!!");
    return;
  }

  Plan plan;
  bool is_success = mg.plan(plan) == MoveItErrorCode::SUCCESS;

  if (is_success)
  {
    mg.move();
  }
  else
  {
    RCLCPP_ERROR(logger, "Forward kinematics planing failed!");
  }
}

// Inverse Kinematics
void move_ik(MoveGroupInterface& mg, geometry_msgs::msg::Pose const& target_pose)
{
  mg.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&mg] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(mg.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    mg.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planing failed!");
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("move");
  RCLCPP_INFO(logger, "Start node");
  auto mg_arm = MoveGroupInterface(node, "panda_arm_hand");

  // fk states
  std::vector<double> prepick_arm_joint_degs = { -3, 77, 2, -60, 3, 164, 38 };
  std::vector<double> standby_arm_joint_degs = { 0, -45, 0, -135, 0, 92, 45 };
  std::vector<double> basket1_arm_joint_degs = { 122, 14, 17, -82, -15, 145, 45 };
  std::vector<double> basket2_arm_joint_degs = { 166, -32, 6, -134, 7, 127, 50 };

  // parse commandline arguments
  std::vector<std::string> parse_values = parseCmdArguments(argc, argv);
  std::string cmd = parse_values.at(0);

  if (cmd == "ik")
  {
    // Inverse Kinematics
    geometry_msgs::msg::Pose target = createGeometryMsg(parse_values);
    auto mg_arm = MoveGroupInterface(node, "panda_arm");
    move_ik(mg_arm, target);
  }
  else
  {
    // Forward Kinematics
    FkJointCommand goal;
    goal.arm_joint_degs = standby_arm_joint_degs;
    goal.gripper_pos_meter = 0.04;

    if (parse_values.size() == 2)
    {
      goal.gripper_pos_meter = std::stod(parse_values.at(1));
    }

    if (cmd == "fk:prepick")
    {
      goal.arm_joint_degs = prepick_arm_joint_degs;
    }
    else if (cmd == "fk:standby")
    {
      goal.arm_joint_degs = standby_arm_joint_degs;
    }
    else if (cmd == "fk:basket1" || cmd == "fk:basket")
    {
      goal.arm_joint_degs = basket1_arm_joint_degs;
    }
    else if (cmd == "fk:basket2")
    {
      basket2_arm_joint_degs.at(0) = 166;
      goal.arm_joint_degs = basket2_arm_joint_degs;
    }
    else if (cmd == "fk:basket3")
    {
      basket1_arm_joint_degs.at(0) = -158;
      goal.arm_joint_degs = basket1_arm_joint_degs;
    }

    move_fk(mg_arm, goal);
  }

  rclcpp::shutdown();
  return 0;
}