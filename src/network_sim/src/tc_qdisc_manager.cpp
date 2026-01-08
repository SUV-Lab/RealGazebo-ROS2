#include "network_sim/tc_qdisc_manager.hpp"

#include <sstream>
#include <iomanip>
#include <thread>
#include <chrono>

namespace network_sim
{

TCQdiscManager::TCQdiscManager(
  const std::string& interface,
  rclcpp::Logger logger)
: interface_(interface),
  logger_(logger),
  initialized_(false)
{
  executor_ = std::make_unique<TCCommandExecutor>(logger);
}

TCQdiscManager::~TCQdiscManager()
{
  if (initialized_) {
    RCLCPP_INFO(logger_, "Cleaning up TC rules on shutdown...");
    cleanup();
  }
}

bool TCQdiscManager::initialize()
{
  // Check if interface exists
  std::string check_cmd = "ip link show " + interface_;
  auto result = executor_->execute(check_cmd);

  if (!result.success) {
    RCLCPP_FATAL(
      logger_,
      "Network interface '%s' not found! Cannot apply TC rules.",
      interface_.c_str());
    return false;
  }

  RCLCPP_INFO(logger_, "Found network interface: %s", interface_.c_str());

  // Remove any existing qdiscs (ignore errors if none exist)
  cleanup();

  // Create root HTB qdisc (handle 1:)
  std::string root_cmd = "tc qdisc add dev " + interface_ +
                         " root handle 1: htb default 999";
  result = executor_->execute(root_cmd);

  if (!result.success) {
    RCLCPP_ERROR(
      logger_,
      "Failed to create root qdisc: %s",
      result.error.c_str());
    return false;
  }

  // Create default class (1:999) for unclassified traffic
  std::string default_class = "tc class add dev " + interface_ +
                              " parent 1: classid 1:999 htb rate 1000mbit";
  result = executor_->execute(default_class);

  if (!result.success) {
    RCLCPP_ERROR(
      logger_,
      "Failed to create default class: %s",
      result.error.c_str());
    return false;
  }

  initialized_ = true;
  RCLCPP_INFO(logger_, "TC qdisc initialized on %s", interface_.c_str());
  return true;
}

bool TCQdiscManager::apply_impairments(
  const std::string& dest_ip,
  int dest_vehicle_id,
  const ImpairmentConfig& config)
{
  if (!initialized_) {
    RCLCPP_ERROR(logger_, "TC qdisc not initialized!");
    return false;
  }

  std::string class_id = get_class_id(dest_vehicle_id);
  std::string handle_id = get_handle_id(dest_vehicle_id);

  // Check if this is a new destination or update to existing
  bool is_new = (active_configs_.find(dest_vehicle_id) == active_configs_.end());

  if (is_new) {
    // Create new class for this destination
    std::string class_cmd = "tc class add dev " + interface_ +
                            " parent 1: classid " + class_id +
                            " htb rate 1000mbit ceil 1000mbit";
    auto result = executor_->execute(class_cmd);

    if (!result.success) {
      RCLCPP_ERROR(
        logger_,
        "Failed to create class for vehicle %d: %s",
        dest_vehicle_id,
        result.error.c_str());
      return false;
    }

    // Create netem qdisc attached to this class
    std::ostringstream netem_cmd;
    netem_cmd << "tc qdisc add dev " << interface_
              << " parent " << class_id
              << " handle " << handle_id << ": netem";

    // Add latency and jitter
    netem_cmd << " delay " << std::fixed << std::setprecision(2)
              << config.latency_ms << "ms "
              << config.jitter_ms << "ms";

    // Add packet loss
    if (config.packet_loss_rate > 0.0) {
      netem_cmd << " loss " << std::fixed << std::setprecision(2)
                << (config.packet_loss_rate * 100.0) << "%";
    }

    result = executor_->execute(netem_cmd.str());

    if (!result.success) {
      RCLCPP_ERROR(
        logger_,
        "Failed to create netem qdisc for vehicle %d: %s",
        dest_vehicle_id,
        result.error.c_str());
      // Clean up the class we just created
      executor_->execute("tc class del dev " + interface_ + " classid " + class_id);
      return false;
    }

    // Create filter to match destination IP
    std::string filter_cmd = "tc filter add dev " + interface_ +
                             " protocol ip parent 1: prio 1 u32 " +
                             "match ip dst " + dest_ip + "/32 " +
                             "flowid " + class_id;
    result = executor_->execute(filter_cmd);

    if (!result.success) {
      RCLCPP_ERROR(
        logger_,
        "Failed to create filter for vehicle %d: %s",
        dest_vehicle_id,
        result.error.c_str());
      // Clean up
      executor_->execute("tc qdisc del dev " + interface_ + " parent " + class_id);
      executor_->execute("tc class del dev " + interface_ + " classid " + class_id);
      return false;
    }

    RCLCPP_INFO(
      logger_,
      "Created TC rules for vehicle %d (IP: %s): latency=%.2fms, jitter=%.2fms, loss=%.1f%%",
      dest_vehicle_id,
      dest_ip.c_str(),
      config.latency_ms,
      config.jitter_ms,
      config.packet_loss_rate * 100.0);

  } else {
    // Update existing netem qdisc
    std::ostringstream netem_cmd;
    netem_cmd << "tc qdisc change dev " << interface_
              << " parent " << class_id
              << " handle " << handle_id << ": netem";

    // Add latency and jitter
    netem_cmd << " delay " << std::fixed << std::setprecision(2)
              << config.latency_ms << "ms "
              << config.jitter_ms << "ms";

    // Add packet loss
    if (config.packet_loss_rate > 0.0) {
      netem_cmd << " loss " << std::fixed << std::setprecision(2)
                << (config.packet_loss_rate * 100.0) << "%";
    }

    auto result = executor_->execute(netem_cmd.str());

    if (!result.success) {
      RCLCPP_ERROR(
        logger_,
        "Failed to update netem qdisc for vehicle %d: %s",
        dest_vehicle_id,
        result.error.c_str());

      // Retry once after a short delay
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      result = executor_->execute(netem_cmd.str());

      if (!result.success) {
        RCLCPP_ERROR(logger_, "Retry also failed, skipping update");
        return false;
      }
    }

    RCLCPP_DEBUG(
      logger_,
      "Updated TC rules for vehicle %d: latency=%.2fms, jitter=%.2fms, loss=%.1f%%",
      dest_vehicle_id,
      config.latency_ms,
      config.jitter_ms,
      config.packet_loss_rate * 100.0);
  }

  // Store configuration
  active_configs_[dest_vehicle_id] = config;

  return true;
}

bool TCQdiscManager::remove_impairments(int dest_vehicle_id)
{
  if (!initialized_) {
    return false;
  }

  auto it = active_configs_.find(dest_vehicle_id);
  if (it == active_configs_.end()) {
    // Not found, nothing to remove
    return true;
  }

  std::string class_id = get_class_id(dest_vehicle_id);

  // Delete filter (need to find and delete by pref)
  // Since we used prio 1, we need to delete all filters and this is complex
  // For simplicity, we'll delete the entire qdisc and class

  // Delete netem qdisc
  std::string qdisc_cmd = "tc qdisc del dev " + interface_ +
                          " parent " + class_id;
  executor_->execute(qdisc_cmd);  // Ignore errors

  // Delete class
  std::string class_cmd = "tc class del dev " + interface_ +
                          " classid " + class_id;
  executor_->execute(class_cmd);  // Ignore errors

  // Note: Filters are automatically removed when class is deleted

  active_configs_.erase(it);

  RCLCPP_INFO(
    logger_,
    "Removed TC rules for vehicle %d",
    dest_vehicle_id);

  return true;
}

bool TCQdiscManager::cleanup()
{
  // Remove all TC qdiscs on interface
  std::string cmd = "tc qdisc del dev " + interface_ + " root";
  auto result = executor_->execute(cmd);
  // Ignore errors - it's okay if there's no qdisc to delete

  active_configs_.clear();
  initialized_ = false;

  RCLCPP_DEBUG(logger_, "TC rules cleaned up on %s", interface_.c_str());

  return true;
}

std::map<int, ImpairmentConfig> TCQdiscManager::get_current_configs() const
{
  return active_configs_;
}

std::string TCQdiscManager::build_dest_ip(int vehicle_id) const
{
  // Destination IP: 172.30.0.{10 + vehicle_id}
  // vehicle_id is the container ID (0, 1, 2, ...)
  return "172.30.0." + std::to_string(10 + vehicle_id);
}

std::string TCQdiscManager::get_class_id(int vehicle_id) const
{
  // Class ID: 1:{10 + vehicle_id}
  return "1:" + std::to_string(10 + vehicle_id);
}

std::string TCQdiscManager::get_handle_id(int vehicle_id) const
{
  // Handle ID: same as class minor ID
  return std::to_string(10 + vehicle_id);
}

}  // namespace network_sim
