#ifndef NETWORK_SIM__TC_CONTROLLER_NODE_HPP_
#define NETWORK_SIM__TC_CONTROLLER_NODE_HPP_

#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

#include "network_sim/tc_qdisc_manager.hpp"
#include "network_sim/communication_model_interface.hpp"

namespace network_sim
{

class TCControllerNode : public rclcpp::Node
{
public:
  explicit TCControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~TCControllerNode() override;

  // Apply communication quality metrics directly (bypassing ROS2 topic)
  void apply_quality_metrics(const std::vector<CommunicationQuality>& qualities);

  // Reset impairments for a vehicle with stale position data
  void reset_impairments_for_stale_vehicle(int vehicle_id);

private:
  // Service callback to enable/disable TC impairments
  void enable_service_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  // Timer callback for publishing diagnostics
  void publish_diagnostics();

  // Check for stale metrics
  void check_stale_metrics();

  // Service for enable/disable
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;

  // Publisher for diagnostics
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_pub_;

  // Timer for diagnostics publishing
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  // Timer for checking stale metrics
  rclcpp::TimerBase::SharedPtr stale_check_timer_;

  // TC qdisc manager
  std::unique_ptr<TCQdiscManager> tc_manager_;

  // State
  int instance_id_;
  bool tc_enabled_;
  std::map<int, rclcpp::Time> last_update_time_;

  // Parameters
  std::string network_interface_;
  double update_interval_sec_;
  bool enable_on_startup_;
  double max_latency_ms_;
  double max_jitter_ms_;
  double max_packet_loss_rate_;
};

}  // namespace network_sim

#endif  // NETWORK_SIM__TC_CONTROLLER_NODE_HPP_
