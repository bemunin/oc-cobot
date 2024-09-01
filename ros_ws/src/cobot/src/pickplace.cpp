#include <rclcpp/rclcpp.hpp>
#include "cobot/mtc_conveyor_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto mtc_node = std::make_shared<cobot::MTCConveyorNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_node]() {
    executor.add_node(mtc_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_node->getNodeBaseInterface());
  });

  mtc_node->setupPlanningScene();
  mtc_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}