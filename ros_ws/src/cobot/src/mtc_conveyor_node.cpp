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
  moveit::planning_interface::PlanningSceneInterface psi;
  moveit_msgs::msg::CollisionObject conveyor;
  moveit_msgs::msg::CollisionObject table;
  moveit_msgs::msg::CollisionObject object;
  moveit_msgs::msg::CollisionObject sensor_emitter;
  moveit_msgs::msg::CollisionObject sensor_receiver;

  // Add conveyor
  conveyor.id = "conveyor";
  conveyor.header.frame_id = "world";
  conveyor.primitives.resize(1);
  conveyor.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  conveyor.primitives[0].dimensions = { 1.0, 4, 0.71 };
  conveyor.operation = moveit_msgs::msg::CollisionObject::ADD;

  geometry_msgs::msg::Pose conveyor_pose;
  conveyor_pose.position.x = 0.82;
  conveyor_pose.position.y = -2 + 1.1329;
  conveyor_pose.position.z = -0.355 - 0.09;
  conveyor_pose.orientation.w = 1;
  conveyor.pose = conveyor_pose;

  // Add table
  table.id = "table";
  table.header.frame_id = "world";
  table.primitives.resize(1);
  table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  table.primitives[0].dimensions = { 1.44, 2.13, 0.8 };
  table.operation = moveit_msgs::msg::CollisionObject::ADD;

  geometry_msgs::msg::Pose table_pose;
  table_pose.position.x = -0.49;
  table_pose.position.y = 0;
  table_pose.position.z = -0.44;
  table_pose.orientation.w = 1;
  table.pose = table_pose;

  // Add sensor emitter
  sensor_emitter.id = "sensor_emitter";
  sensor_emitter.header.frame_id = "world";
  sensor_emitter.primitives.resize(1);
  sensor_emitter.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  sensor_emitter.primitives[0].dimensions = { 0.04, 0.07, 0.04 };
  conveyor.operation = moveit_msgs::msg::CollisionObject::ADD;

  geometry_msgs::msg::Pose sensor_emitter_pose;
  sensor_emitter_pose.position.x = 0.34;
  sensor_emitter_pose.position.y = -0.03;
  sensor_emitter_pose.position.z = -0.07;
  sensor_emitter_pose.orientation.w = 1;
  sensor_emitter.pose = sensor_emitter_pose;

  // Add sensor receiver
  sensor_receiver.id = "sensor_receiver";
  sensor_receiver.header.frame_id = "world";
  sensor_receiver.primitives.resize(1);
  sensor_receiver.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  sensor_receiver.primitives[0].dimensions = { 0.04, 0.07, 0.04 };
  conveyor.operation = moveit_msgs::msg::CollisionObject::ADD;

  geometry_msgs::msg::Pose sensor_receiver_pose;
  sensor_receiver_pose.position.x = 1.29;
  sensor_receiver_pose.position.y = -0.03;
  sensor_receiver_pose.position.z = -0.07;
  sensor_receiver_pose.orientation.w = 1;
  sensor_receiver.pose = sensor_receiver_pose;

  // Add object
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.16, 0.02 };

  geometry_msgs::msg::Pose obj_pose;
  obj_pose.position.x = 0.82;
  obj_pose.position.y = -0.03;
  obj_pose.position.z = -0.005;  // -0.06
  obj_pose.orientation.w = 1.0;
  object.pose = obj_pose;

  addBasketCollisionObj(psi, "basket1", -0.74, 0.46, 0.185);
  addBasketCollisionObj(psi, "basket2", -0.74, 0.0, 0.185);
  addBasketCollisionObj(psi, "basket3", -0.74, -0.46, 0.185);

  psi.applyCollisionObject(conveyor);
  psi.applyCollisionObject(sensor_emitter);
  psi.applyCollisionObject(sensor_receiver);
  psi.applyCollisionObject(object);
  psi.applyCollisionObject(table);
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

  // // place object container
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

// private: Scene setup

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

mtc::Stage::pointer MTCConveyorNode::approachObjectStage(std::string stage_name)
{
  auto stage = std::make_unique<mtc::stages::MoveRelative>(stage_name, cartesian_planner_);
  stage->properties().set("marker_ns", "approach_object");
  stage->properties().set("link", hand_frame_);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.05, 0.08);

  // Set hand forward direction
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = hand_frame_;
  vec.vector.z = 1.0;
  stage->setDirection(vec);
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

  Eigen::Vector3d custom_axis(0.0, 0.0, 1.0);
  custom_axis.normalize();
  stage->setRotationAxis(custom_axis);

  Eigen::Isometry3d grasp_frame_transform;
  Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  grasp_frame_transform.linear() = q.matrix();
  grasp_frame_transform.translation().z() = 0.1;
  // Compute IK
  auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
  wrapper->setMaxIKSolutions(10);
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

mtc::Stage::pointer MTCConveyorNode::liftObjectStage(std::string stage_name)
{
  auto stage = std::make_unique<mtc::stages::MoveRelative>(stage_name, cartesian_planner_);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.1, 0.2);
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
  target_pose_msg.pose.position.x = -0.05;
  target_pose_msg.pose.position.y = 0.3;
  target_pose_msg.pose.position.z = 0.0;
  target_pose_msg.pose.orientation.w = 1.0;
  stage->setPose(target_pose_msg);
  stage->setMonitoredStage(attach_object_stage_);  // Hook into attach_object_stage

  // Compute IK
  auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
  wrapper->setMaxIKSolutions(10);
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
  stage->setMinMaxDistance(0, 0.2);
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
void MTCConveyorNode::addBasketCollisionObj(moveit::planning_interface::PlanningSceneInterface& psi,
                                            std::string frame_id, double x, double y, double z)
{
  moveit_msgs::msg::CollisionObject basket;
  basket.id = frame_id;
  basket.header.frame_id = "world";
  basket.primitives.resize(1);
  basket.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  basket.primitives[0].dimensions = { 0.6, 0.42, 0.37 };
  basket.operation = moveit_msgs::msg::CollisionObject::ADD;

  geometry_msgs::msg::Pose basket_pose;
  basket_pose.position.x = x;
  basket_pose.position.y = y;
  basket_pose.position.z = z;
  basket.pose = basket_pose;

  psi.applyCollisionObject(basket);
}

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