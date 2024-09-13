#ifndef COBOT_CONVEYOR_TASK_NODE_H
#define COBOT_CONVEYOR_TASK_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("cobot");
namespace mtc = moveit::task_constructor;
using PipelinePlanner = mtc::solvers::PipelinePlanner;
using JointInterpolationPlanner = mtc::solvers::JointInterpolationPlanner;
using CartesianPath = mtc::solvers::CartesianPath;
using GroupPlannerVector = mtc::stages::Connect::GroupPlannerVector;

namespace cobot
{
class MTCConveyorNode
{
public:
  MTCConveyorNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  rclcpp::Node::SharedPtr node_;
  mtc::Task task_;
  mtc::Stage* current_state_ptr_;
  mtc::Stage* attach_object_stage_;

  const std::string arm_group_name_ = "panda_arm";
  const std::string hand_group_name_ = "hand";
  const std::string hand_frame_ = "panda_hand";

  std::shared_ptr<PipelinePlanner> sampling_planner_;
  std::shared_ptr<JointInterpolationPlanner> interpolation_planner_;
  std::shared_ptr<CartesianPath> cartesian_planner_;

  void setupTaskStages();

  // Utility functions
  void addBasketCollisionObj(moveit::planning_interface::PlanningSceneInterface& psi, std::string frame_id, double x,
                             double y, double z);
  void addToContainer(mtc::ContainerBase::pointer& container, mtc::Stage::pointer&& stage);
  void addToTask(mtc::Stage::pointer&& stage);
  void addToTask(mtc::ContainerBase::pointer& stage);
  mtc::ContainerBase::pointer createSerialContainer(std::string container_name);

  // stages
  mtc::Stage::pointer currentStateStage(std::string stage_name);
  mtc::Stage::pointer handStage(std::string stage_name, std::string goal);
  mtc::Stage::pointer connectStage(std::string stage_name, GroupPlannerVector group_planner_vector);
  mtc::Stage::pointer toHomeStage(std::string stage_name);
  mtc::Stage::pointer approachObjectStage(std::string stage_name);
  mtc::Stage::pointer generateGraspPoseStage(std::string stage_name, const std::string& target_object);
  mtc::Stage::pointer allowCollisionStage(std::string stage_name, const std::string& target_object, bool is_allow);
  mtc::Stage::pointer graspObjectStage(std::string stage_name, const std::string& target_object, bool is_attach);
  mtc::Stage::pointer liftObjectStage(std::string stage_name);
  mtc::Stage::pointer generatePlacePoseStage(std::string stage_name, const std::string& target_object);
  mtc::Stage::pointer retreatStage(std::string stage_name);
};

}  // namespace cobot

#endif  // COBOT_CONVEYOR_TASK_NODE_H
