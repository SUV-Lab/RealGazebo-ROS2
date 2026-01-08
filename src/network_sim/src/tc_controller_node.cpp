#include "network_sim/tc_controller_node.hpp"

#include <chrono>
#include <algorithm>

namespace network_sim
{

TCControllerNode::TCControllerNode(const rclcpp::NodeOptions & options)
: Node("tc_controller", options)
{
  // Declare and get parameters
  this->declare_parameter("instance_id", 0);
  this->declare_parameter("network_interface", "eth1");
  this->declare_parameter("enable_on_startup", true);
  this->declare_parameter("update_interval", 1.0);
  this->declare_parameter("max_latency_ms", 1000.0);
  this->declare_parameter("max_jitter_ms", 500.0);
  this->declare_parameter("max_packet_loss_rate", 0.99);

  instance_id_ = this->get_parameter("instance_id").as_int();
  network_interface_ = this->get_parameter("network_interface").as_string();
  enable_on_startup_ = this->get_parameter("enable_on_startup").as_bool();
  update_interval_sec_ = this->get_parameter("update_interval").as_double();
  max_latency_ms_ = this->get_parameter("max_latency_ms").as_double();
  max_jitter_ms_ = this->get_parameter("max_jitter_ms").as_double();
  max_packet_loss_rate_ = this->get_parameter("max_packet_loss_rate").as_double();

  tc_enabled_ = enable_on_startup_;

  RCLCPP_INFO(
    this->get_logger(),
    "TC Controller Node initialized for instance_id=%d, interface=%s",
    instance_id_,
    network_interface_.c_str());

  RCLCPP_INFO(
    this->get_logger(),
    "Safety limits: latency=%.1fms, jitter=%.1fms, packet_loss=%.1f%%",
    max_latency_ms_,
    max_jitter_ms_,
    max_packet_loss_rate_ * 100.0);

  // Create TC qdisc manager
  tc_manager_ = std::make_unique<TCQdiscManager>(network_interface_, this->get_logger());

  // Initialize TC qdisc hierarchy
  if (!tc_manager_->initialize()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize TC qdisc! Exiting.");
    rclcpp::shutdown();
    return;
  }

  // Create service for enable/disable
  std::string service_name = "~/enable";
  enable_service_ = this->create_service<std_srvs::srv::SetBool>(
    service_name,
    std::bind(
      &TCControllerNode::enable_service_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(
    this->get_logger(),
    "Service '%s' ready",
    service_name.c_str());

  // Create diagnostics publisher
  diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "diagnostics",
    rclcpp::QoS(10));

  // Create timers
  diagnostics_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(update_interval_sec_),
    std::bind(&TCControllerNode::publish_diagnostics, this));

  stale_check_timer_ = this->create_wall_timer(
    std::chrono::seconds(5),
    std::bind(&TCControllerNode::check_stale_metrics, this));

  if (tc_enabled_) {
    RCLCPP_INFO(this->get_logger(), "TC impairments ENABLED on startup");
  } else {
    RCLCPP_INFO(this->get_logger(), "TC impairments DISABLED on startup");
  }
}

TCControllerNode::~TCControllerNode()
{
  RCLCPP_INFO(this->get_logger(), "TC Controller Node shutting down...");
}

void TCControllerNode::apply_quality_metrics(
  const std::vector<CommunicationQuality>& qualities)
{
  if (!tc_enabled_) {
    return;  // TC is disabled, ignore metrics
  }

  for (const auto& quality : qualities) {
    // Apply safety limits (clamping)
    double clamped_latency = std::min(quality.latency_ms, max_latency_ms_);
    double clamped_jitter = std::min(quality.jitter_ms, max_jitter_ms_);
    double clamped_plr = std::min(quality.packet_loss_rate, max_packet_loss_rate_);

    // Build impairment config
    ImpairmentConfig config{
      clamped_latency,
      clamped_jitter,
      clamped_plr
    };

    // Build destination IP
    // Note: dest_vehicle_id is ROS2 namespace ID (1, 2, 3...)
    // Container mapping: vehicle{N} has IP 172.30.0.{10+N}
    // ROS2 namespace vehicle{M} = container vehicle{M-1}
    // Formula: IP = 172.30.0.{9 + dest_vehicle_id}
    int dest_container_id = quality.dest_vehicle_id - 1;
    std::string dest_ip = "172.30.0." + std::to_string(10 + dest_container_id);

    // Apply TC impairments
    tc_manager_->apply_impairments(dest_ip, dest_container_id, config);

    // Update last update time
    last_update_time_[quality.dest_vehicle_id] = this->now();

    // Log to terminal (user requested)
    RCLCPP_INFO(this->get_logger(),
      "Applied to vehicle %d: latency=%.1fms, jitter=%.1fms, PLR=%.1f%%",
      quality.dest_vehicle_id, clamped_latency, clamped_jitter, clamped_plr * 100);
  }
}

void TCControllerNode::enable_service_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  bool old_state = tc_enabled_;
  tc_enabled_ = request->data;

  if (tc_enabled_ && !old_state) {
    // Enabling TC
    response->success = true;
    response->message = "TC impairments enabled";
    RCLCPP_INFO(this->get_logger(), "TC impairments ENABLED via service");
  } else if (!tc_enabled_ && old_state) {
    // Disabling TC - remove all rules
    auto configs = tc_manager_->get_current_configs();
    for (const auto& [vehicle_id, config] : configs) {
      tc_manager_->remove_impairments(vehicle_id);
    }
    response->success = true;
    response->message = "TC impairments disabled, all rules removed";
    RCLCPP_INFO(this->get_logger(), "TC impairments DISABLED via service");
  } else {
    // No change
    response->success = true;
    response->message = tc_enabled_ ? "TC already enabled" : "TC already disabled";
  }
}

void TCControllerNode::publish_diagnostics()
{
  auto diag_array = diagnostic_msgs::msg::DiagnosticArray();
  diag_array.header.stamp = this->now();

  auto status = diagnostic_msgs::msg::DiagnosticStatus();
  status.name = "tc_controller_vehicle" + std::to_string(instance_id_);
  status.hardware_id = network_interface_;

  if (tc_enabled_) {
    auto configs = tc_manager_->get_current_configs();

    if (configs.empty()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "TC enabled but no destinations configured";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "TC impairments active: " + std::to_string(configs.size()) + " destinations";

      // Add per-destination metrics
      for (const auto& [vehicle_id, config] : configs) {
        std::string prefix = "vehicle_" + std::to_string(vehicle_id) + "_";

        diagnostic_msgs::msg::KeyValue kv_latency;
        kv_latency.key = prefix + "latency_ms";
        kv_latency.value = std::to_string(config.latency_ms);
        status.values.push_back(kv_latency);

        diagnostic_msgs::msg::KeyValue kv_jitter;
        kv_jitter.key = prefix + "jitter_ms";
        kv_jitter.value = std::to_string(config.jitter_ms);
        status.values.push_back(kv_jitter);

        diagnostic_msgs::msg::KeyValue kv_loss;
        kv_loss.key = prefix + "packet_loss";
        kv_loss.value = std::to_string(config.packet_loss_rate);
        status.values.push_back(kv_loss);
      }
    }
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "TC impairments disabled";
  }

  diag_array.status.push_back(status);
  diagnostics_pub_->publish(diag_array);
}

void TCControllerNode::check_stale_metrics()
{
  auto now = this->now();

  for (const auto& [dest_id, last_time] : last_update_time_) {
    auto age = (now - last_time).seconds();

    if (age > 5.0 && age < 30.0) {
      // Warn about stale data (throttled to every 10 seconds)
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        10000,  // 10 seconds
        "Metrics for vehicle %d are stale (%.1f seconds old)",
        dest_id,
        age);
    }
  }
}

void TCControllerNode::reset_impairments_for_stale_vehicle(int vehicle_id)
{
  if (!tc_enabled_) {
    return;  // TC is disabled, nothing to reset
  }

  // Remove TC rules for this vehicle to allow fresh position updates
  if (tc_manager_->remove_impairments(vehicle_id)) {
    RCLCPP_INFO(
      this->get_logger(),
      "Reset TC rules for vehicle %d due to stale position data",
      vehicle_id);

    // Remove from last_update_time tracking
    last_update_time_.erase(vehicle_id);
  }
}

}  // namespace network_sim
