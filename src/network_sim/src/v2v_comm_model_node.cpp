#include "network_sim/v2v_comm_model_node.hpp"
#include "network_sim/log_distance_model.hpp"
#include "network_sim/congestion_aware_model.hpp"
#include "network_sim/tc_controller_node.hpp"

#include <chrono>
#include <functional>
#include <cmath>
#include <sstream>

namespace network_sim
{

V2VCommModelNode::V2VCommModelNode(
  const rclcpp::NodeOptions & options,
  std::weak_ptr<TCControllerNode> tc_controller)
: Node("v2v_comm_model", options),
  tc_controller_(tc_controller)
{
  // Declare and get parameters
  this->declare_parameter("instance_id", 0);
  this->declare_parameter("distance_calculation_interval", 1.0);
  this->declare_parameter("gz_world_name", "c-track");
  this->declare_parameter("vehicle_models", "");

  this->declare_parameter("comm_model_type", "log_distance");
  this->declare_parameter("tx_power_dbm", 20.0);
  this->declare_parameter("path_loss_exponent", 2.5);
  this->declare_parameter("max_retransmission_delay_ms", 50.0);
  this->declare_parameter("max_jitter_ms", 20.0);
  this->declare_parameter("baseline_latency_ms", 2.0);
  this->declare_parameter("baseline_jitter_ms", 0.5);

  this->declare_parameter("enable_congestion_model", false);
  this->declare_parameter("rssi_range_threshold_dbm", -90.0);
  this->declare_parameter("congestion_plr_factor", 0.4);
  this->declare_parameter("congestion_plr_alpha", 0.2);
  this->declare_parameter("congestion_latency_beta", 0.4);
  this->declare_parameter("congestion_jitter_gamma", 0.3);

  instance_id_ = this->get_parameter("instance_id").as_int();
  distance_calculation_interval_ = this->get_parameter("distance_calculation_interval").as_double();
  gz_world_name_ = this->get_parameter("gz_world_name").as_string();

  // Parse vehicle_models: "x500_0,lc_62_1,boat_8" -> map<name, id>
  std::string models_str = this->get_parameter("vehicle_models").as_string();
  if (!models_str.empty()) {
    std::istringstream ss(models_str);
    std::string token;
    while (std::getline(ss, token, ',')) {
      // Extract ID from last underscore: "x500_0" -> 0, "lc_62_1" -> 1
      auto last_underscore = token.rfind('_');
      if (last_underscore != std::string::npos) {
        int vid = std::stoi(token.substr(last_underscore + 1));
        vehicle_model_map_[token] = vid;
      }
    }
  }

  if (vehicle_model_map_.empty()) {
    RCLCPP_WARN(this->get_logger(),
      "No vehicle_models parameter provided. Pose tracking may match incorrect models.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Tracking %zu vehicle models:", vehicle_model_map_.size());
    for (const auto & [name, vid] : vehicle_model_map_) {
      RCLCPP_INFO(this->get_logger(), "  %s -> vehicle_id=%d", name.c_str(), vid);
    }
  }

  comm_model_type_ = this->get_parameter("comm_model_type").as_string();
  tx_power_dbm_ = this->get_parameter("tx_power_dbm").as_double();
  path_loss_exponent_ = this->get_parameter("path_loss_exponent").as_double();
  max_retransmission_delay_ms_ = this->get_parameter("max_retransmission_delay_ms").as_double();
  max_jitter_ms_ = this->get_parameter("max_jitter_ms").as_double();
  baseline_latency_ms_ = this->get_parameter("baseline_latency_ms").as_double();
  baseline_jitter_ms_ = this->get_parameter("baseline_jitter_ms").as_double();

  enable_congestion_model_ = this->get_parameter("enable_congestion_model").as_bool();
  rssi_range_threshold_dbm_ = this->get_parameter("rssi_range_threshold_dbm").as_double();
  congestion_plr_factor_ = this->get_parameter("congestion_plr_factor").as_double();
  congestion_plr_alpha_ = this->get_parameter("congestion_plr_alpha").as_double();
  congestion_latency_beta_ = this->get_parameter("congestion_latency_beta").as_double();
  congestion_jitter_gamma_ = this->get_parameter("congestion_jitter_gamma").as_double();

  RCLCPP_INFO(
    this->get_logger(),
    "Parameters: instance_id=%d, distance_calculation_interval=%.1fs, gz_world=%s",
    instance_id_, distance_calculation_interval_, gz_world_name_.c_str());

  RCLCPP_INFO(
    this->get_logger(),
    "Communication: model=%s, tx_power=%.1fdBm, path_loss_exp=%.1f",
    comm_model_type_.c_str(), tx_power_dbm_, path_loss_exponent_);

  // Initialize communication model
  initialize_communication_model();

  // Subscribe to Gazebo dynamic_pose/info via gz-transport
  std::string gz_topic = "/world/" + gz_world_name_ + "/dynamic_pose/info";
  if (!gz_node_.Subscribe(gz_topic, &V2VCommModelNode::gz_pose_callback, this)) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Failed to subscribe to Gazebo topic: %s",
      gz_topic.c_str());
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Subscribed to Gazebo topic: %s (via gazebo-network, TC-free)",
      gz_topic.c_str());
  }

  // Create timer for periodic distance calculation
  distance_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(distance_calculation_interval_),
    std::bind(&V2VCommModelNode::calculate_and_apply_distances, this));
}

void V2VCommModelNode::gz_pose_callback(const gz::msgs::Pose_V & msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);

  for (int i = 0; i < msg.pose_size(); ++i) {
    const auto & pose = msg.pose(i);
    const std::string & name = pose.name();

    int vid = lookup_vehicle_id(name);
    if (vid < 0) {
      continue;  // Not a known vehicle model
    }

    latest_poses_[vid] = VehiclePose{
      pose.position().x(),
      pose.position().y(),
      pose.position().z()
    };
  }
}

int V2VCommModelNode::lookup_vehicle_id(const std::string & model_name)
{
  auto it = vehicle_model_map_.find(model_name);
  if (it != vehicle_model_map_.end()) {
    return it->second;
  }
  return -1;  // Not a known vehicle model
}

void V2VCommModelNode::calculate_and_apply_distances()
{
  std::lock_guard<std::mutex> lock(pose_mutex_);

  // Check if reference vehicle (this container's vehicle) exists
  if (latest_poses_.find(instance_id_) == latest_poses_.end()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Waiting for reference vehicle %d pose from Gazebo...",
      instance_id_);
    return;
  }

  const auto & ref_pose = latest_poses_[instance_id_];

  // Pre-calculate distance threshold from RSSI threshold
  double distance_threshold_m = 1.0 * pow(10.0,
    (tx_power_dbm_ - rssi_range_threshold_dbm_ - 40.0) / (10.0 * path_loss_exponent_));

  // Count vehicles in communication range
  int vehicles_in_range = 1;  // Include reference vehicle itself
  for (const auto & [vid, pose] : latest_poses_) {
    if (vid == instance_id_) continue;
    double dist = calculate_euclidean_distance(
      ref_pose.x, ref_pose.y, ref_pose.z,
      pose.x, pose.y, pose.z);
    if (dist <= distance_threshold_m) {
      vehicles_in_range++;
    }
  }

  // Print header
  RCLCPP_INFO(
    this->get_logger(),
    "\n=== Distances from Vehicle %d (Gazebo pose) ===\n"
    "    Vehicles in Range: %d (threshold: %.1f dBm, %.0f m)",
    instance_id_,
    vehicles_in_range,
    rssi_range_threshold_dbm_,
    distance_threshold_m);

  // Calculate communication quality for all other vehicles
  std::vector<CommunicationQuality> qualities;

  for (const auto & [vid, pose] : latest_poses_) {
    if (vid == instance_id_) {
      RCLCPP_INFO(this->get_logger(), "  Vehicle %d: [REFERENCE]", vid);
      continue;
    }

    double distance = calculate_euclidean_distance(
      ref_pose.x, ref_pose.y, ref_pose.z,
      pose.x, pose.y, pose.z);

    auto quality = comm_model_->calculate(distance, vehicles_in_range);

    // dest_vehicle_id uses ROS2 convention (instance_id + 1)
    quality.source_vehicle_id = instance_id_ + 1;
    quality.dest_vehicle_id = vid + 1;

    RCLCPP_INFO(
      this->get_logger(),
      "  Vehicle %d:\n"
      "    Distance: %.2f m\n"
      "    RSSI: %.1f dBm\n"
      "    Packet Loss Rate: %.1f%%\n"
      "    Latency: %.2f ms\n"
      "    Jitter: %.2f ms",
      vid,
      quality.distance_m,
      quality.rssi_dbm,
      quality.packet_loss_rate * 100.0,
      quality.latency_ms,
      quality.jitter_ms);

    qualities.push_back(quality);
  }

  // Apply to TC controller
  if (!qualities.empty()) {
    if (auto tc = tc_controller_.lock()) {
      tc->apply_quality_metrics(qualities);
    }
  }
}

double V2VCommModelNode::calculate_euclidean_distance(
  double x1, double y1, double z1,
  double x2, double y2, double z2)
{
  double dx = x2 - x1;
  double dy = y2 - y1;
  double dz = z2 - z1;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
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
      max_jitter_ms_,
      baseline_latency_ms_,
      baseline_jitter_ms_
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

}  // namespace network_sim
