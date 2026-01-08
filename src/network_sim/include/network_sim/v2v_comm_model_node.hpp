#ifndef NETWORK_SIM__V2V_COMM_MODEL_NODE_HPP_
#define NETWORK_SIM__V2V_COMM_MODEL_NODE_HPP_

#include <map>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "network_sim/communication_model_interface.hpp"

namespace network_sim
{

class TCControllerNode;  // Forward declaration

class V2VCommModelNode : public rclcpp::Node
{
public:
  explicit V2VCommModelNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    std::weak_ptr<TCControllerNode> tc_controller = {});

private:
  // Scan for vehicle_global_position topics and subscribe to new ones
  void scan_and_subscribe_topics();

  // Callback for vehicle global position messages (stores position data only)
  void vehicle_global_position_callback(
    int vehicle_id,
    const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);

  // Calculate and print distances from reference vehicle (timer callback)
  void calculate_and_print_distances();

  // Calculate Haversine distance between two lat/lon coordinates (in meters)
  double calculate_haversine_distance(
    double lat1, double lon1, double lat2, double lon2);

  // Initialize communication model based on type
  void initialize_communication_model();

  // Handle stale position data
  void handle_stale_position(int vehicle_id, double age_sec);

  // Timers
  rclcpp::TimerBase::SharedPtr scan_timer_;
  rclcpp::TimerBase::SharedPtr distance_timer_;

  // Map of vehicle_id to subscription
  std::map<int, rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr> subscriptions_;

  // Map of vehicle_id to latest position
  std::map<int, px4_msgs::msg::VehicleGlobalPosition::SharedPtr> latest_positions_;

  // Map of vehicle_id to timestamp when position was last updated
  std::map<int, rclcpp::Time> position_timestamps_;

  // Communication model
  std::unique_ptr<CommunicationModelInterface> comm_model_;

  // Parameters
  double scan_interval_sec_;
  double distance_calculation_interval_;
  int reference_vehicle_id_;
  double stale_position_timeout_sec_;

  // Communication model parameters
  std::string comm_model_type_;
  double tx_power_dbm_;
  double path_loss_exponent_;
  double max_retransmission_delay_ms_;
  double max_jitter_ms_;

  // Congestion model parameters
  bool enable_congestion_model_;
  double rssi_range_threshold_dbm_;
  double congestion_plr_factor_;
  double congestion_plr_alpha_;
  double congestion_latency_beta_;
  double congestion_jitter_gamma_;

  // State flags
  bool reference_found_;
  bool reference_warning_shown_;

  // Direct reference to TC controller for internal communication
  std::weak_ptr<TCControllerNode> tc_controller_;
};

}  // namespace network_sim

#endif  // NETWORK_SIM__V2V_COMM_MODEL_NODE_HPP_