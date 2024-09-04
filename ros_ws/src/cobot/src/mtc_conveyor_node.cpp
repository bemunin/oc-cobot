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

// Public
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

// Private: Main
void MTCConveyorNode::setupTaskStages()
{
  std::string target_object = "object";
  GroupPlannerVector to_pick_group_planner{ { arm_group_name_, sampling_planner_ } };
  GroupPlannerVector to_place_group_planner{ { arm_group_name_, sampling_planner_ },
                                             { hand_group_name_, sampling_planner_ } };

  addToTask(currentStateStage("current"));
  addToTask(handStage("open hand", "open"));

  addToTask(connectStage("move to pick", to_pick_group_planner));

  // pick object container
  {
    auto pick = createSerialContainer("pick object");

    addToContainer(pick, approachObjectStage("approach object"));
    addToContainer(pick, generateGraspPoseStage("generate grasp pose", target_object));
    addToContainer(pick, allowCollisionStage("allow collision (hand,object)", target_object, true));
    addToContainer(pick, handStage("close hand", "close"));
    addToContainer(pick, graspObjectStage("attach object", target_object, true));
    addToContainer(pick, liftObjectStage("lift object"));

    addToTask(pick);
  }

  addToTask(connectStage("move to place", to_place_group_planner));

  // place object container
  {
    auto place = createSerialContainer("place object");

    addToContainer(place, generatePlacePoseStage("generate place pose", target_object));
    addToContainer(place, handStage("open hand", "open"));
    addToContainer(place, allowCollisionStage("forbid collision (hand,object)", target_object, false));
    addToContainer(place, graspObjectStage("detach object", target_object, false));
    addToContainer(place, retreatStage("retreat"));

    addToTask(place);
  }

  addToTask(toHomeStage("return home"));
}

// Private: Stages factory methods
mtc::Stage::pointer MTCConveyorNode::handStage(std::string stage_name, std::string goal)
{
  auto stage = std::make_unique<mtc::stages::MoveTo>(stage_name, interpolation_planner_);
  stage->setGroup(hand_group_name_);
  stage->setGoal(goal);
  return stage;
}

mtc::Stage::pointer MTCConveyorNode::currentStateStage(std::string stage_name)
{
  auto stage = std::make_unique<mtc::stages::CurrentState>(stage_name);
  current_state_ptr_ = stage.get();
  return stage;
}

mtc::Stage::pointer MTCConveyorNode::connectStage(std::string stage_name, GroupPlannerVector group_planner_vector)
{
  auto stage = std::make_unique<mtc::stages::Connect>(stage_name, group_planner_vector);
  stage->setTimeout(5.0);
  stage->properties().configureInitFrom(mtc::Stage::PARENT);
  return stage;
}

mtc::Stage::pointer MTCConveyorNode::toHomeStage(std::string stage_name)
{
  auto stage = std::make_unique<mtc::stages::MoveTo>(stage_name, interpolation_planner_);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setGoal("ready");
  return stage;
}

mtc::Stage::pointer MTCConveyorNode::generateGraspPoseStage(std::string stage_name, const std::string& target_object)
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

  return wrapper;
}

mtc::Stage::pointer MTCConveyorNode::allowCollisionStage(std::string stage_name, const std::string& target_object,
                                                         bool is_allow)
{
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(stage_name);
  stage->allowCollisions(
      target_object,
      task_.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(), is_allow);
  return stage;
}

mtc::Stage::pointer MTCConveyorNode::graspObjectStage(std::string stage_name, const std::string& target_object,
                                                      bool is_attach)
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
  return stage;
}

mtc::Stage::pointer MTCConveyorNode::approachObjectStage(std::string stage_name)
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
  return stage;
}

mtc::Stage::pointer MTCConveyorNode::liftObjectStage(std::string stage_name)
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
  return stage;
}

mtc::Stage::pointer MTCConveyorNode::generatePlacePoseStage(std::string stage_name, const std::string& target_object)
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

  return wrapper;
}

mtc::Stage::pointer MTCConveyorNode::retreatStage(std::string stage_name)
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

  return stage;
}

// Private: Utiltiy functions
void MTCConveyorNode::addToContainer(mtc::ContainerBase::pointer& container, mtc::Stage::pointer&& stage)
{
  container->insert(std::move(stage));
}

void MTCConveyorNode::addToTask(mtc::Stage::pointer&& stage)
{
  task_.add(std::move(stage));
}

void MTCConveyorNode::addToTask(mtc::ContainerBase::pointer& container)
{
  task_.add(std::move(container));
}

mtc::ContainerBase::pointer MTCConveyorNode::createSerialContainer(std::string container_name)
{
  auto container = std::make_unique<mtc::SerialContainer>(container_name);
  task_.properties().exposeTo(container->properties(), { "eef", "group", "ik_frame" });
  container->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

  return container;
}

}  // namespace cobot