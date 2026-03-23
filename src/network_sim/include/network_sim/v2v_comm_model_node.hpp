#ifndef NETWORK_SIM__V2V_COMM_MODEL_NODE_HPP_
#define NETWORK_SIM__V2V_COMM_MODEL_NODE_HPP_

#include <map>
#include <set>
#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>

#include "network_sim/communication_model_interface.hpp"

namespace network_sim
{

class TCControllerNode;  // Forward declaration

struct VehiclePose
{
  double x;
  double y;
  double z;
};

class V2VCommModelNode : public rclcpp::Node
{
public:
  explicit V2VCommModelNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    std::weak_ptr<TCControllerNode> tc_controller = {});

private:
  // Callback for Gazebo dynamic_pose/info topic
  void gz_pose_callback(const gz::msgs::Pose_V & msg);

  // Calculate and apply distances from reference vehicle (timer callback)
  void calculate_and_apply_distances();

  // Calculate Euclidean distance between two 3D positions (in meters)
  double calculate_euclidean_distance(
    double x1, double y1, double z1,
    double x2, double y2, double z2);

  // Initialize communication model based on type
  void initialize_communication_model();

  // Look up vehicle ID from model name using the known vehicle_models map
  int lookup_vehicle_id(const std::string & model_name);

  // Timer for periodic distance calculation
  rclcpp::TimerBase::SharedPtr distance_timer_;

  // Gazebo transport node and subscriber
  gz::transport::Node gz_node_;

  // Map of vehicle_id to latest pose (protected by mutex)
  std::map<int, VehiclePose> latest_poses_;
  std::mutex pose_mutex_;

  // Communication model
  std::unique_ptr<CommunicationModelInterface> comm_model_;

  // Known vehicle model names: model_name -> vehicle_id
  std::map<std::string, int> vehicle_model_map_;

  // Parameters
  double distance_calculation_interval_;
  int instance_id_;
  std::string gz_world_name_;

  // Communication model parameters
  std::string comm_model_type_;
  double tx_power_dbm_;
  double path_loss_exponent_;
  double max_retransmission_delay_ms_;
  double max_jitter_ms_;
  double baseline_latency_ms_;
  double baseline_jitter_ms_;

  // Congestion model parameters
  bool enable_congestion_model_;
  double rssi_range_threshold_dbm_;
  double congestion_plr_factor_;
  double congestion_plr_alpha_;
  double congestion_latency_beta_;
  double congestion_jitter_gamma_;

  // Direct reference to TC controller for internal communication
  std::weak_ptr<TCControllerNode> tc_controller_;
};

}  // namespace network_sim

#endif  // NETWORK_SIM__V2V_COMM_MODEL_NODE_HPP_
