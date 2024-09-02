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

void MTCConveyorNode::addCurrentStateStage()
{
  auto stage = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr_ = stage.get();
  task_.add(std::move(stage));
}

void MTCConveyorNode::addHandStage(std::string stage_name, std::string goal)
{
  auto stage = std::make_unique<mtc::stages::MoveTo>(stage_name, interpolation_planner_);
  stage->setGroup(hand_group_name_);
  stage->setGoal(goal);

  task_.add(std::move(stage));
}

void MTCConveyorNode::addHandStage(std::string stage_name, std::string goal,
                                   std::unique_ptr<mtc::SerialContainer>& container)
{
  auto stage = std::make_unique<mtc::stages::MoveTo>(stage_name, interpolation_planner_);
  stage->setGroup(hand_group_name_);
  stage->setGoal(goal);
  container->insert(std::move(stage));
}

// pick object container
void MTCConveyorNode::addConnectStage(std::string stage_name, GroupPlannerVector group_planner_vector)
{
  auto stage = std::make_unique<mtc::stages::Connect>(stage_name, group_planner_vector);
  stage->setTimeout(5.0);
  stage->properties().configureInitFrom(mtc::Stage::PARENT);
  task_.add(std::move(stage));
}

std::unique_ptr<mtc::SerialContainer> MTCConveyorNode::createSerialContainer(std::string container_name)
{
  auto container = std::make_unique<mtc::SerialContainer>(container_name);
  task_.properties().exposeTo(container->properties(), { "eef", "group", "ik_frame" });
  container->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

  return container;
}

void MTCConveyorNode::addApproachObjectStage(std::string stage_name, std::unique_ptr<mtc::SerialContainer>& container)
{
  auto stage = std::make_unique<mtc::stages::MoveRelative>(stage_name, cartesian_planner_);
  stage->properties().set("marker_ns", "approach_object");
  stage->properties().set("link", hand_frame_);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.1, 0.15);

  // Set hand forward direction
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = hand_frame_;
  vec.vector.z = 1.0;
  stage->setDirection(vec);
  container->insert(std::move(stage));
}

void MTCConveyorNode::addGenerateGraspPoseStage(std::string stage_name, const std::string& target_object,
                                                std::unique_ptr<mtc::SerialContainer>& container)
{
  // Sample grasp pose
  auto stage = std::make_unique<mtc::stages::GenerateGraspPose>(stage_name);
  stage->properties().configureInitFrom(mtc::Stage::PARENT);
  stage->properties().set("marker_ns", "grasp_pose");
  stage->setPreGraspPose("open");
  stage->setObject(target_object);
  stage->setAngleDelta(M_PI / 12);
  stage->setMonitoredStage(current_state_ptr_);  // Hook into current state

  Eigen::Isometry3d grasp_frame_transform;
  Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  grasp_frame_transform.linear() = q.matrix();
  grasp_frame_transform.translation().z() = 0.1;
  // Compute IK
  auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
  wrapper->setMaxIKSolutions(8);
  wrapper->setMinSolutionDistance(1.0);
  wrapper->setIKFrame(grasp_frame_transform, hand_frame_);
  wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  container->insert(std::move(wrapper));
}

void MTCConveyorNode::addAllowCollisionToObjectStage(std::string stage_name, bool is_allow,
                                                     const std::string& target_object,
                                                     std::unique_ptr<mtc::SerialContainer>& container)
{
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(stage_name);
  stage->allowCollisions(
      target_object,
      task_.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(), is_allow);
  container->insert(std::move(stage));
}

void MTCConveyorNode::addGraspObjectStage(std::string stage_name, bool is_attach, const std::string& target_object,
                                          std::unique_ptr<mtc::SerialContainer>& container)
{
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(stage_name);
  if (is_attach)
  {
    stage->attachObject(target_object, hand_frame_);
    attach_object_stage_ = stage.get();
  }
  else
  {
    stage->detachObject(target_object, hand_frame_);
  }
  container->insert(std::move(stage));
}

void MTCConveyorNode::addLiftObjectStage(std::string stage_name, std::unique_ptr<mtc::SerialContainer>& container)
{
  auto stage = std::make_unique<mtc::stages::MoveRelative>(stage_name, cartesian_planner_);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.1, 0.3);
  stage->setIKFrame(hand_frame_);
  stage->properties().set("marker_ns", "lift_object");

  // Set upward direction
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "world";
  vec.vector.z = 1.0;
  stage->setDirection(vec);
  container->insert(std::move(stage));
}

void MTCConveyorNode::addGeneratePlacePoseStage(std::string stage_name, const std::string& target_object,
                                                std::unique_ptr<mtc::SerialContainer>& container)
{
  auto stage = std::make_unique<mtc::stages::GeneratePlacePose>(stage_name);
  stage->properties().configureInitFrom(mtc::Stage::PARENT);
  stage->properties().set("marker_ns", "place_pose");
  stage->setObject(target_object);

  geometry_msgs::msg::PoseStamped target_pose_msg;
  target_pose_msg.header.frame_id = target_object;
  target_pose_msg.pose.position.y = 0.5;
  target_pose_msg.pose.orientation.w = 1.0;
  stage->setPose(target_pose_msg);
  stage->setMonitoredStage(attach_object_stage_);  // Hook into attach_object_stage

  // Compute IK
  auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
  wrapper->setMaxIKSolutions(2);
  wrapper->setMinSolutionDistance(1.0);
  wrapper->setIKFrame(target_object);
  wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  container->insert(std::move(wrapper));
}

void MTCConveyorNode::addRetreatStage(std::string stage_name, std::unique_ptr<mtc::SerialContainer>& container)
{
  auto stage = std::make_unique<mtc::stages::MoveRelative>(stage_name, cartesian_planner_);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.1, 0.3);
  stage->setIKFrame(hand_frame_);
  stage->properties().set("marker_ns", "retreat");

  // Set retreat direction
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "world";
  vec.vector.x = -0.5;
  stage->setDirection(vec);
  container->insert(std::move(stage));
}

void MTCConveyorNode::addReturnHome(std::string stage_name)
{
  auto stage = std::make_unique<mtc::stages::MoveTo>(stage_name, interpolation_planner_);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setGoal("ready");
  task_.add(std::move(stage));
}

void MTCConveyorNode::setupTaskStages()
{
  std::string target_object = "object";

  addCurrentStateStage();
  addHandStage("open hand", "open");

  GroupPlannerVector to_pick_group_planner{ { arm_group_name_, sampling_planner_ } };
  addConnectStage("move to pick", to_pick_group_planner);

  // pick object container
  {
    auto pick = createSerialContainer("pick object");
    addApproachObjectStage("approach object", pick);
    addGenerateGraspPoseStage("generate grasp pose", target_object, pick);
    addAllowCollisionToObjectStage("allow collision (hand,object)", true, target_object, pick);
    addHandStage("close hand", "close", pick);
    addGraspObjectStage("attach object", true, target_object, pick);
    addLiftObjectStage("lift object", pick);
    task_.add(std::move(pick));
  }

  GroupPlannerVector to_place_group_planner{ { arm_group_name_, sampling_planner_ },
                                             { hand_group_name_, sampling_planner_ } };
  addConnectStage("move to place", to_place_group_planner);

  // place object container
  {
    auto place = createSerialContainer("place object");

    addGeneratePlacePoseStage("generate place pose", target_object, place);
    addHandStage("open hand", "open", place);
    addAllowCollisionToObjectStage("forbid collision (hand,object)", false, target_object, place);
    addGraspObjectStage("detach object", false, target_object, place);
    addRetreatStage("retreat", place);
    task_.add(std::move(place));
  }
  addReturnHome("return home");
}
}  // namespace cobot