cmake_minimum_required(VERSION 3.8)
project(network_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Include directories
include_directories(include)

# Vehicle Network Simulator executable
add_executable(network_sim_node
  src/main.cpp
  src/v2v_comm_model_node.cpp
  src/tc_controller_node.cpp
  src/tc_qdisc_manager.cpp
  src/tc_command_executor.cpp
  src/log_distance_model.cpp
  src/congestion_aware_model.cpp
)

ament_target_dependencies(network_sim_node
  rclcpp
  std_srvs
  diagnostic_msgs
)

target_link_libraries(network_sim_node
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

# Install executable
install(TARGETS network_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
