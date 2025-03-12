#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/planning_scene/planning_scene.h>

using namespace moveit::task_constructor;

Task createTask(const rclcpp::Node::SharedPtr& node)
{
  Task t;

  t.stages()->setName("Cartesian Path");

  RCLCPP_INFO(node->get_logger(), "Creating Cartesian Path task");

  const std::string arm = "panda_arm";
  const std::string arm_with_gripper = "panda_arm_hand";

  auto cartesian_interpolation = std::make_shared<solvers::CartesianPath>();
  auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();

  t.loadRobotModel(node);
  RCLCPP_INFO(node->get_logger(), "Loaded robot model: %s", t.getRobotModel()->getName().c_str());

  auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());

  {
    RCLCPP_INFO(node->get_logger(), "Create initial state");
    auto& state = scene->getCurrentStateNonConst();
    state.setToDefaultValues(state.getJointModelGroup(arm), "ready");

    auto fixed = std::make_unique<stages::FixedState>("initial state");
    fixed->setState(scene);
    t.add(std::move(fixed));
    RCLCPP_INFO(node->get_logger(), "Added initial state to the task");
  }

  // stage 1
  {
    RCLCPP_INFO(node->get_logger(), "Creating stage: Move 0.05m in +x direction");
    auto stage = std::make_unique<stages::MoveRelative>("x +0.05", cartesian_interpolation);
    stage->setGroup(arm);
    stage->setIKFrame("panda_link7");
    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = "base_link";
    direction.vector.x = 0.05;
    stage->setDirection(direction);
    t.add(std::move(stage));
    RCLCPP_INFO(node->get_logger(), "Added stage: Move 0.05m in +x direction");
  }

  // stage 2
  {
    RCLCPP_INFO(node->get_logger(), "Creating stage: Move 0.02m in -y direction");
    auto stage = std::make_unique<stages::MoveRelative>("y -0.02", cartesian_interpolation);
    stage->setGroup(arm);
    stage->setIKFrame("panda_link7");
    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = "base_link";

    // Set the y component to -0.02 meters (move 2 cm in the negative y direction)
    direction.vector.y = -0.02;
    stage->setDirection(direction);

    t.add(std::move(stage));
    RCLCPP_INFO(node->get_logger(), "Added -y movement stage to task");
  }

  // stage 3
  {
    RCLCPP_INFO(node->get_logger(), "Creating stage: Rotate -18 degrees around z-axis");

    auto stage = std::make_unique<stages::MoveRelative>("rz -18°", cartesian_interpolation);
    stage->setGroup(arm);
    stage->setIKFrame("panda_link7");

    // Create a TwistStamped message to specify a rotation
    geometry_msgs::msg::TwistStamped twist;
    twist.header.frame_id = "base_link";

    // Set the angular z component to -pi/10 radians (-18 degrees)
    twist.twist.angular.z = -M_PI / 10.;

    stage->setDirection(twist);
    t.add(std::move(stage));
    RCLCPP_INFO(node->get_logger(), "Added rotation stage to task");
  }

  // stage 4
  {
    RCLCPP_INFO(node->get_logger(), "Creating stage: Move joints by specified angles");
    auto stage = std::make_unique<stages::MoveRelative>("joint offset", cartesian_interpolation);
    stage->setGroup(arm);

    // Create a map of joint names to angle offsets
    std::map<std::string, double> offsets = {
      { "panda_joint2", M_PI / 12. },  // Rotate this joint by 15 degrees (pi/12 radians)
      { "panda_joint4", -M_PI / 12. }  // Rotate this joint by -15 degrees
    };

    stage->setDirection(offsets);

    t.add(std::move(stage));
    RCLCPP_INFO(node->get_logger(), "Added joint offset stage to task");
  }

  // stage 5
  {
    RCLCPP_INFO(node->get_logger(), "Creating connect stage");

    // Create a vector of groups and their associated planners
    stages::Connect::GroupPlannerVector planners = { { arm, joint_interpolation } };

    // Create a Connect stage to smoothly link the previous stages
    auto connect = std::make_unique<stages::Connect>("connect", planners);
    t.add(std::move(connect));
    RCLCPP_INFO(node->get_logger(), "Added connect stage to task");
  }

  // Set the final state of the robot

  {
    RCLCPP_INFO(node->get_logger(), "Setting final state");
    auto fixed = std::make_unique<stages::FixedState>("final state");
    fixed->setState(scene);
    t.add(std::move(fixed));
    RCLCPP_INFO(node->get_logger(), "Added final state to task");
  }

  RCLCPP_INFO(node->get_logger(), "Task creation completed");

  return t;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cartesian_demo");

  RCLCPP_INFO(node->get_logger(), "Starting Cartesian path planning demo");

  std::thread spinning_thread([node] { rclcpp::spin(node); });

  auto task = createTask(node);

  try
  {
    if (task.plan())
    {
      RCLCPP_INFO(node->get_logger(), "Task planning completed successfully");
      task.introspection().publishSolution(*task.solutions().front());
      RCLCPP_INFO(node->get_logger(), "Solution published");
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "Task planning failed");
    }
  }
  catch (const InitStageException& e)
  {
    RCLCPP_ERROR(node->get_logger(), "Initialization failed: %s", e.what());
    RCLCPP_ERROR(node->get_logger(), "Task name: %s", task.name().c_str());
  }

  RCLCPP_INFO(node->get_logger(), "Waiting for ROS 2 spinning thread to finish");

  spinning_thread.join();

  RCLCPP_INFO(node->get_logger(), "Cartesian path planning demo completed");

  return 0;
}