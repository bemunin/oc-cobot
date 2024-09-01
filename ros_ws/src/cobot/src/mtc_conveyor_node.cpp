#include "cobot/mtc_conveyor_node.hpp"

namespace cobot
{
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCConveyorNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCConveyorNode::MTCConveyorNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_conveyor_node", options) }, current_state_ptr_(nullptr)
{
  // initialize planners
  sampling_planner_ = std::make_shared<PipelinePlanner>(node_);
  interpolation_planner_ = std::make_shared<JointInterpolationPlanner>();

  cartesian_planner_ = std::make_shared<CartesianPath>();
  cartesian_planner_->setMaxVelocityScalingFactor(1.0);
  cartesian_planner_->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner_->setStepSize(.01);

  // setup task
  task_.stages()->setName("conveyor task");
  task_.loadRobotModel(node_);

  task_.setProperty("group", arm_group_name_);
  task_.setProperty("eef", hand_group_name_);
  task_.setProperty("ik_frame", hand_frame_);
}

void MTCConveyorNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = -0.25;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void MTCConveyorNode::doTask()
{
  setupTaskStages();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

// Stages

void MTCConveyorNode::setupTaskStages()
{
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr_ = stage_state_current.get();
  task_.add(std::move(stage_state_current));

  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner_);
  stage_open_hand->setGroup(hand_group_name_);
  stage_open_hand->setGoal("open");
  task_.add(std::move(stage_open_hand));
}
}  // namespace cobot