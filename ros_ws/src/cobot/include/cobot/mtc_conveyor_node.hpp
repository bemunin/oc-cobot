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
  void addToContainer(mtc::ContainerBase::pointer& container, mtc::Stage::pointer&& stage);
  void addToTask(mtc::Stage::pointer&& stage);
  void addToTask(mtc::ContainerBase::pointer& stage);

  // stages
  mtc::Stage::pointer currentStateStage(std::string stage_name);
  mtc::Stage::pointer handStage(std::string stage_name, std::string goal);

  // stages

  void addConnectStage(std::string stage_name, GroupPlannerVector group_planner_vector);
  mtc::ContainerBase::pointer createSerialContainer(std::string container_name);
  void addApproachObjectStage(std::string stage_name, mtc::ContainerBase::pointer& container);
  void addGenerateGraspPoseStage(std::string stage_name, const std::string& target_object,
                                 mtc::ContainerBase::pointer& container);
  void addAllowCollisionToObjectStage(std::string stage_name, bool is_allow, const std::string& target_object,
                                      mtc::ContainerBase::pointer& container);

  void addGraspObjectStage(std::string stage_name, bool is_attach, const std::string& target_object,
                           mtc::ContainerBase::pointer& container);
  void addLiftObjectStage(std::string stage_name, mtc::ContainerBase::pointer& container);

  void addGeneratePlacePoseStage(std::string stage_name, const std::string& target_object,
                                 mtc::ContainerBase::pointer& container);

  void addRetreatStage(std::string stage_name, mtc::ContainerBase::pointer& container);

  void addReturnHome(std::string stage_name);
};

}  // namespace cobot

#endif  // COBOT_CONVEYOR_TASK_NODE_H
