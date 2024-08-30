#include "cobot/mtc_conveyor_node.hpp"

namespace cobot
{
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCConveyorNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCConveyorNode::MTCConveyorNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_conveyor_node", options) }
{
}

void MTCConveyorNode::setupPlanningScene()
{
}

void MTCConveyorNode::doTask()
{
}

mtc::Task MTCConveyorNode::createTask()
{
  mtc::Task task;
  return task;
}
}  // namespace cobot