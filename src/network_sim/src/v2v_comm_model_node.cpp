#include "network_sim/v2v_comm_model_node.hpp"
#include "network_sim/log_distance_model.hpp"
#include "network_sim/congestion_aware_model.hpp"
#include "network_sim/tc_controller_node.hpp"

#include <regex>
#include <chrono>
#include <functional>
#include <cmath>

namespace network_sim
{

V2VCommModelNode::V2VCommModelNode(
  const rclcpp::NodeOptions & options,
  std::weak_ptr<TCControllerNode> tc_controller)
: Node("v2v_comm_model", options),
  tc_controller_(tc_controller)
{
  // Declare and get parameters
  this->declare_parameter("scan_interval", 2.0);
  this->declare_parameter("reference_vehicle_id", 1);  // Default to 1 (maps to /vehicle1)
  this->declare_parameter("distance_calculation_interval", 1.0);
  this->declare_parameter("stale_position_timeout_sec", 5.0);

  this->declare_parameter("comm_model_type", "log_distance");
  this->declare_parameter("tx_power_dbm", 20.0);
  this->declare_parameter("path_loss_exponent", 2.5);
  this->declare_parameter("max_retransmission_delay_ms", 50.0);
  this->declare_parameter("max_jitter_ms", 20.0);

  this->declare_parameter("enable_congestion_model", true);
  this->declare_parameter("rssi_range_threshold_dbm", -90.0);
  this->declare_parameter("congestion_plr_factor", 0.4);
  this->declare_parameter("congestion_plr_alpha", 0.2);
  this->declare_parameter("congestion_latency_beta", 0.4);
  this->declare_parameter("congestion_jitter_gamma", 0.3);

  scan_interval_sec_ = this->get_parameter("scan_interval").as_double();
  reference_vehicle_id_ = this->get_parameter("reference_vehicle_id").as_int();
  distance_calculation_interval_ = this->get_parameter("distance_calculation_interval").as_double();
  stale_position_timeout_sec_ = this->get_parameter("stale_position_timeout_sec").as_double();

  comm_model_type_ = this->get_parameter("comm_model_type").as_string();
  tx_power_dbm_ = this->get_parameter("tx_power_dbm").as_double();
  path_loss_exponent_ = this->get_parameter("path_loss_exponent").as_double();
  max_retransmission_delay_ms_ = this->get_parameter("max_retransmission_delay_ms").as_double();
  max_jitter_ms_ = this->get_parameter("max_jitter_ms").as_double();

  enable_congestion_model_ = this->get_parameter("enable_congestion_model").as_bool();
  rssi_range_threshold_dbm_ = this->get_parameter("rssi_range_threshold_dbm").as_double();
  congestion_plr_factor_ = this->get_parameter("congestion_plr_factor").as_double();
  congestion_plr_alpha_ = this->get_parameter("congestion_plr_alpha").as_double();
  congestion_latency_beta_ = this->get_parameter("congestion_latency_beta").as_double();
  congestion_jitter_gamma_ = this->get_parameter("congestion_jitter_gamma").as_double();

  // Initialize state flags
  reference_found_ = false;
  reference_warning_shown_ = false;

  RCLCPP_INFO(
    this->get_logger(),
    "Parameters: scan_interval=%.1fs, reference_vehicle_id=%d, distance_calculation_interval=%.1fs",
    scan_interval_sec_, reference_vehicle_id_, distance_calculation_interval_);

  RCLCPP_INFO(
    this->get_logger(),
    "Communication: model=%s, tx_power=%.1fdBm, path_loss_exp=%.1f",
    comm_model_type_.c_str(), tx_power_dbm_, path_loss_exponent_);

  // Initialize communication model
  initialize_communication_model();

  // Create timer for periodic topic scanning
  scan_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(scan_interval_sec_),
    std::bind(&V2VCommModelNode::scan_and_subscribe_topics, this));

  // Create timer for periodic distance calculation
  distance_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(distance_calculation_interval_),
    std::bind(&V2VCommModelNode::calculate_and_print_distances, this));

  // Perform initial scan
  scan_and_subscribe_topics();
}

void V2VCommModelNode::scan_and_subscribe_topics()
{
  // Get all active topics
  auto topic_names_and_types = this->get_topic_names_and_types();

  // Regular expression to match /vehicleN/fmu/out/vehicle_global_position
  std::regex topic_pattern(R"(/vehicle(\d+)/fmu/out/vehicle_global_position)");
  std::smatch match;

  // Scan for vehicle_global_position topics
  for (const auto & [topic_name, topic_types] : topic_names_and_types) {
    if (std::regex_match(topic_name, match, topic_pattern)) {
      // Extract vehicle ID from topic name
      int vehicle_id = std::stoi(match[1].str());

      // Check if we're already subscribed to this vehicle
      if (subscriptions_.find(vehicle_id) != subscriptions_.end()) {
        continue;  // Already subscribed
      }

      // Create new subscription for this vehicle
      RCLCPP_INFO(
        this->get_logger(),
        "Found and subscribing to: %s (Vehicle ID: %d)",
        topic_name.c_str(),
        vehicle_id);

      auto subscription = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        topic_name,
        rclcpp::SensorDataQoS(),
        [this, vehicle_id](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
          this->vehicle_global_position_callback(vehicle_id, msg);
        });

      // Store subscription in map
      subscriptions_[vehicle_id] = subscription;
    }
  }
}

void V2VCommModelNode::vehicle_global_position_callback(
  int vehicle_id,
  const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
  // Store latest position data
  latest_positions_[vehicle_id] = msg;

  // Record timestamp when position was updated
  position_timestamps_[vehicle_id] = this->now();
}

void V2VCommModelNode::calculate_and_print_distances()
{
  // Check if reference vehicle exists
  if (latest_positions_.find(reference_vehicle_id_) == latest_positions_.end()) {
    if (!reference_warning_shown_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Waiting for reference vehicle %d...",
        reference_vehicle_id_);
      reference_warning_shown_ = true;
    }
    return;
  }

  // Mark reference vehicle as found and notify user
  if (!reference_found_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Reference vehicle %d found! Starting distance calculations.",
      reference_vehicle_id_);
    reference_found_ = true;
  }

  // Get reference position
  auto ref_pos = latest_positions_[reference_vehicle_id_];

  // Prepare vector for direct communication with TC controller
  std::vector<CommunicationQuality> qualities;

  // Count vehicles in communication range
  int vehicles_in_range = 1;  // Include reference vehicle itself

  // Pre-calculate distance threshold from RSSI threshold for performance
  // RSSI = TxPower - (PL0 + 10*n*log10(d/d0))
  // Solving for d: d = d0 * 10^((TxPower - RSSI - PL0)/(10*n))
  double distance_threshold_m = 1.0 * pow(10.0,
    (tx_power_dbm_ - rssi_range_threshold_dbm_ - 40.0) / (10.0 * path_loss_exponent_));

  for (const auto & [vid, pos] : latest_positions_) {
    if (vid == reference_vehicle_id_) continue;

    // Check if position data is stale
    auto now = this->now();
    auto timestamp_it = position_timestamps_.find(vid);

    if (timestamp_it != position_timestamps_.end()) {
      double age_sec = (now - timestamp_it->second).seconds();

      if (age_sec > stale_position_timeout_sec_) {
        // Position is stale - reset TC rules to allow reconnection
        handle_stale_position(vid, age_sec);
        continue;  // Skip this vehicle in distance calculations
      }
    }

    double dist = calculate_haversine_distance(
      ref_pos->lat, ref_pos->lon, pos->lat, pos->lon);

    // Count if within distance threshold
    if (dist <= distance_threshold_m) {
      vehicles_in_range++;
    }
  }

  // Print header
  RCLCPP_INFO(
    this->get_logger(),
    "\n=== Distances and Communication Quality from Reference Vehicle %d ===\n"
    "    Vehicles in Range: %d (threshold: %.1f dBm, %.0f m)",
    reference_vehicle_id_,
    vehicles_in_range,
    rssi_range_threshold_dbm_,
    distance_threshold_m);

  // Calculate and print distances and communication quality for all vehicles
  for (const auto & [vehicle_id, pos] : latest_positions_) {
    if (vehicle_id == reference_vehicle_id_) {
      RCLCPP_INFO(this->get_logger(), "  Vehicle %d: [REFERENCE]", vehicle_id);
    } else {
      // Calculate distance
      double distance = calculate_haversine_distance(
        ref_pos->lat, ref_pos->lon,
        pos->lat, pos->lon);

      // Calculate communication quality
      auto quality = comm_model_->calculate(distance, vehicles_in_range);

      // Set vehicle IDs
      quality.source_vehicle_id = reference_vehicle_id_;
      quality.dest_vehicle_id = vehicle_id;

      // Print comprehensive results
      RCLCPP_INFO(
        this->get_logger(),
        "  Vehicle %d:\n"
        "    Distance: %.2f m\n"
        "    RSSI: %.1f dBm\n"
        "    Packet Loss Rate: %.1f%%\n"
        "    Latency: %.2f ms\n"
        "    Jitter: %.2f ms",
        vehicle_id,
        quality.distance_m,
        quality.rssi_dbm,
        quality.packet_loss_rate * 100.0,
        quality.latency_ms,
        quality.jitter_ms);

      // Add to qualities vector for direct TC controller communication
      qualities.push_back(quality);
    }
  }

  // Directly call TC controller (bypassing ROS2 topic)
  if (!qualities.empty()) {
    if (auto tc = tc_controller_.lock()) {
      tc->apply_quality_metrics(qualities);

      // Log summary (user requested)
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Vehicle %d: Calculated quality for %zu vehicles, applied to TC controller",
        reference_vehicle_id_, qualities.size());
    }
  }
}

double V2VCommModelNode::calculate_haversine_distance(
  double lat1, double lon1, double lat2, double lon2)
{
  const double R = 6371000.0;  // Earth radius in meters

  double lat1_rad = lat1 * M_PI / 180.0;
  double lat2_rad = lat2 * M_PI / 180.0;
  double delta_lat = (lat2 - lat1) * M_PI / 180.0;
  double delta_lon = (lon2 - lon1) * M_PI / 180.0;

  double a = sin(delta_lat / 2.0) * sin(delta_lat / 2.0) +
             cos(lat1_rad) * cos(lat2_rad) *
             sin(delta_lon / 2.0) * sin(delta_lon / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

  return R * c;  // Distance in meters
}

void V2VCommModelNode::initialize_communication_model()
{
  // Create base communication model
  std::unique_ptr<CommunicationModelInterface> base_model;

  if (comm_model_type_ == "log_distance") {
    base_model = std::make_unique<LogDistanceModel>(
      tx_power_dbm_,
      path_loss_exponent_,
      1.0,  // reference_distance_m
      40.0, // reference_path_loss_db
      max_retransmission_delay_ms_,
      max_jitter_ms_
    );
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "Unknown communication model type: %s",
      comm_model_type_.c_str());
    throw std::runtime_error("Invalid communication model type");
  }

  // Wrap with congestion model if enabled
  if (enable_congestion_model_) {
    comm_model_ = std::make_unique<CongestionAwareModel>(
      std::move(base_model),
      rssi_range_threshold_dbm_,
      congestion_plr_factor_,
      congestion_plr_alpha_,
      congestion_latency_beta_,
      congestion_jitter_gamma_
    );
    RCLCPP_INFO(
      this->get_logger(),
      "Using communication model: %s",
      comm_model_->get_model_name().c_str());
  } else {
    comm_model_ = std::move(base_model);
    RCLCPP_INFO(
      this->get_logger(),
      "Using communication model: %s (congestion disabled)",
      comm_model_->get_model_name().c_str());
  }
}

void V2VCommModelNode::handle_stale_position(int vehicle_id, double age_sec)
{
  // Reset TC rules for this vehicle to allow reconnection
  if (auto tc = tc_controller_.lock()) {
    tc->reset_impairments_for_stale_vehicle(vehicle_id);

    // Log at INFO level, throttled to every 10 seconds
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      10000,  // 10 seconds
      "Position data for vehicle %d is stale (%.1f seconds old). "
      "Reset TC rules to allow reconnection.",
      vehicle_id,
      age_sec);
  }
}

}  // namespace network_sim