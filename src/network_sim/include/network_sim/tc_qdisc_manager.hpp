#ifndef NETWORK_SIM__TC_QDISC_MANAGER_HPP_
#define NETWORK_SIM__TC_QDISC_MANAGER_HPP_

#include <map>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "network_sim/tc_command_executor.hpp"

namespace network_sim
{

struct ImpairmentConfig {
  double latency_ms;
  double jitter_ms;
  double packet_loss_rate;
};

class TCQdiscManager
{
public:
  TCQdiscManager(
    const std::string& interface,
    rclcpp::Logger logger);

  ~TCQdiscManager();

  // Initialize TC qdisc hierarchy on interface
  bool initialize();

  // Apply or update impairments for a specific destination IP
  bool apply_impairments(
    const std::string& dest_ip,
    int dest_vehicle_id,
    const ImpairmentConfig& config);

  // Remove impairments for a specific destination
  bool remove_impairments(int dest_vehicle_id);

  // Clean up all TC rules
  bool cleanup();

  // Get current configuration
  std::map<int, ImpairmentConfig> get_current_configs() const;

private:
  // Build destination IP from vehicle ID (container ID, not ROS2 namespace)
  std::string build_dest_ip(int vehicle_id) const;

  // Get flow ID (class ID) for vehicle
  std::string get_class_id(int vehicle_id) const;

  // Get handle ID for vehicle
  std::string get_handle_id(int vehicle_id) const;

  std::string interface_;
  rclcpp::Logger logger_;
  std::unique_ptr<TCCommandExecutor> executor_;

  // Map of vehicle_id -> impairment config
  std::map<int, ImpairmentConfig> active_configs_;

  bool initialized_;
};

}  // namespace network_sim

#endif  // NETWORK_SIM__TC_QDISC_MANAGER_HPP_
