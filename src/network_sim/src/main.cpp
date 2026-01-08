#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "network_sim/v2v_comm_model_node.hpp"
#include "network_sim/tc_controller_node.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Create executor with intra-process communication support
  rclcpp::executors::SingleThreadedExecutor executor;

  // Create node options with intra-process comms enabled
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  // Create TC controller first
  auto tc_node = std::make_shared<network_sim::TCControllerNode>(options);

  // Create V2V node with TC controller reference (weak_ptr)
  auto v2v_node = std::make_shared<network_sim::V2VCommModelNode>(
    options,
    std::weak_ptr<network_sim::TCControllerNode>(tc_node));

  // Add nodes to executor
  executor.add_node(v2v_node);
  executor.add_node(tc_node);

  RCLCPP_INFO(
    rclcpp::get_logger("network_sim"),
    "Starting Network Simulator with direct internal communication");

  // Spin
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
