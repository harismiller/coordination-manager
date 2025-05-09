cmake_minimum_required(VERSION 3.8)
project(coordination-manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ltl_automaton_msgs REQUIRED)
find_package(interfaces_hmm_sim REQUIRED)

# Find SQLite3
find_package(SQLite3 REQUIRED)

# ==========================
# System Compiler Test Node
# ==========================
add_executable(system_compiler_test 
  src/system_compiler_test_node.cpp
  src/system_compiler.cpp
  src/environment_compiler.cpp
  src/agent_data.cpp
)
target_include_directories(system_compiler_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(system_compiler_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  system_compiler_test
  "rclcpp"
  "std_msgs"
)

# ==========================
# System Compiler Node
# ==========================
add_executable(system_compiler_node
  src/system_compiler_node.cpp
  src/system_compiler.cpp
  src/environment_compiler.cpp
  src/agent_data.cpp
)
target_include_directories(system_compiler_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(system_compiler_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  system_compiler_node
  "rclcpp"
  "std_msgs"
)
target_link_libraries(system_compiler_node SQLite::SQLite3)

# ==========================
# Behavior Tree Framework
# ==========================
add_executable(behavior_tree_test
  src/BT/bt_control.cpp
  src/BT/behavior_tree_test.cpp
  src/BT/nodes/action_node.cpp
  src/BT/nodes/condition_node.cpp
  src/BT/nodes/sequence_node.cpp
  src/BT/nodes/parallel_node.cpp
)
target_include_directories(behavior_tree_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/BT> 
  $<INSTALL_INTERFACE:include>)
target_compile_features(behavior_tree_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  behavior_tree_test 
  "rclcpp"
)

# ==========================
# Planner Parser Node
# ==========================
add_executable(planner_parser_node
  src/planner_parser_node.cpp
)
target_include_directories(planner_parser_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(planner_parser_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  planner_parser_node
  "rclcpp"
  "std_msgs"
  "ltl_automaton_msgs"
  "geometry_msgs"
  "interfaces_hmm_sim"
)
target_link_libraries(planner_parser_node SQLite::SQLite3)

# ==========================
# Status Agent Fail Publisher Node
# ==========================
add_executable(status_agent_fail_publisher
  src/planner_integration_test_node.cpp
)
target_include_directories(status_agent_fail_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(status_agent_fail_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  status_agent_fail_publisher
  "rclcpp"
  "geometry_msgs"
  "interfaces_hmm_sim"
)

# ==========================
# Installation
# ==========================
install(TARGETS 
  behavior_tree_test
  system_compiler_test
  system_compiler_node
  planner_parser_node
  status_agent_fail_publisher
  DESTINATION lib/${PROJECT_NAME})

# ==========================
# Install Launch Files
# ==========================
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
