cmake_minimum_required(VERSION 3.8)
project(cobot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)

add_executable(move src/move.cpp)
ament_target_dependencies(move rclcpp moveit_ros_planning_interface)

add_executable(pickplace src/pickplace.cpp src/mtc_conveyor_node.cpp)
ament_target_dependencies(pickplace moveit_task_constructor_core rclcpp)
target_include_directories(pickplace PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(pickplace PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  move
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  pickplace
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
