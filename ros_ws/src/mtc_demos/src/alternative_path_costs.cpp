#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/cost_terms.h>
#include <iostream>

using namespace moveit::task_constructor;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node = rclcpp::Node::make_shared("alternative_path_costs_demo", node_options);
  std::thread spinning_thread([node] { rclcpp::spin(node); });

  Task t;
  t.stages()->setName("alternative path costs");

  t.loadRobotModel(node);

  // Ensure the correct robot model is loaded
  assert(t.getRobotModel()->getName() == "panda");

  // Create a planning scene
  auto scene{ std::make_shared<planning_scene::PlanningScene>(t.getRobotModel()) };

  // Get the current robot state and set it to default values
  auto& robot_state{ scene->getCurrentStateNonConst() };
  robot_state.setToDefaultValues();
  robot_state.setToDefaultValues(robot_state.getJointModelGroup("panda_arm"), "home");

  // Create and add the initial state to the task
  // This step gives MoveIt 2 a clear starting point from which to plan the robot's movements.
  auto initial{ std::make_unique<stages::FixedState>("start") };
  initial->setState(scene);
  t.add(std::move(initial));

  // Create a pipeline planner
  // The "pipeline" part means it can chain together multiple planning attempts or strategies.
  // If one method fails, it can try another
  auto pipeline{ std::make_shared<solvers::PipelinePlanner>(node) };

  auto alternatives{ std::make_unique<Alternatives>("connect") };

  // Strategy 1: Minimize path length
  {
    auto connect{ std::make_unique<stages::Connect>(
        "path length", stages::Connect::GroupPlannerVector{ { "panda_arm_hand", pipeline } }) };
    connect->setCostTerm(std::make_unique<cost::PathLength>());  //  Typically in radians, representing the total
                                                                 //  angular displacement of all joints.
    alternatives->add(std::move(connect));
    std::cout << "Added 'path length' strategy" << std::endl;
  }

  // Strategy 2: Minimize trajectory duration
  {
    auto connect{ std::make_unique<stages::Connect>(
        "trajectory duration", stages::Connect::GroupPlannerVector{ { "panda_arm_hand", pipeline } }) };
    connect->setCostTerm(std::make_unique<cost::TrajectoryDuration>());  // Time it would take in seconds to execute the
                                                                         // planned motion.
    alternatives->add(std::move(connect));
    std::cout << "Added 'trajectory duration' strategy" << std::endl;
  }

  // Strategy 3: Minimize end-effector motion
  {
    auto connect{ std::make_unique<stages::Connect>(
        "eef motion", stages::Connect::GroupPlannerVector{ { "panda_arm_hand", pipeline } }) };
    connect->setCostTerm(std::make_unique<cost::LinkMotion>("link6_flange"));  // Distance traveled by the link in
                                                                               // meters
    alternatives->add(std::move(connect));
    std::cout << "Added 'end-effector motion' strategy" << std::endl;
  }

  {
    auto connect{ std::make_unique<stages::Connect>(
        "elbow motion", stages::Connect::GroupPlannerVector{ { "panda_arm_hand", pipeline } }) };
    connect->setCostTerm(std::make_unique<cost::LinkMotion>("link3"));
    alternatives->add(std::move(connect));
    std::cout << "Added 'elbow motion' strategy" << std::endl;
  }

  t.add(std::move(alternatives));
  std::cout << "Added all strategies in the Alternatives Container to the task" << std::endl;

  auto goal_scene{ scene->diff() };
  goal_scene->getCurrentStateNonConst().setToDefaultValues(robot_state.getJointModelGroup("panda_arm_hand"), "ready");

  // Create and add the goal state to the task
  auto goal = std::make_unique<stages::FixedState>("goal");
  goal->setState(goal_scene);
  t.add(std::move(goal));

  // Plan the task
  std::cout << "Starting task planning..." << std::endl;
  try
  {
    t.plan(0);  // The 0 parameter means it will generate as many solutions as possible
    std::cout << "Task planning completed successfully" << std::endl;

    // Print the results
    std::cout << "Planning results:" << std::endl;
    t.printState();
  }
  catch (const InitStageException& e)
  {
    std::cout << "Task planning failed: " << e << std::endl;
  }

  // Keep the node alive for interactive inspection in RViz
  std::cout << "Keeping node alive for RViz inspection. Press Ctrl+C to exit." << std::endl;
  spinning_thread.join();

  return 0;
}