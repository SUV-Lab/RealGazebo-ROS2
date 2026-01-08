#ifndef NETWORK_SIM__COMMUNICATION_MODEL_INTERFACE_HPP_
#define NETWORK_SIM__COMMUNICATION_MODEL_INTERFACE_HPP_

#include <string>

namespace network_sim
{

// Communication quality metrics
struct CommunicationQuality {
  int source_vehicle_id;       // Source vehicle ID
  int dest_vehicle_id;         // Destination vehicle ID
  double distance_m;           // Distance in meters
  double path_loss_db;         // Path loss in dB
  double rssi_dbm;             // Received Signal Strength Indicator in dBm
  double packet_loss_rate;     // Packet loss rate (0.0 ~ 1.0)
  double latency_ms;           // Total latency in milliseconds
  double jitter_ms;            // Jitter in milliseconds
  int vehicles_in_range;       // Number of vehicles in communication range
};

// Abstract base class for communication models
class CommunicationModelInterface {
public:
  virtual ~CommunicationModelInterface() = default;

  // Calculate communication quality based on distance
  virtual CommunicationQuality calculate(double distance_m, int vehicles_in_range = 1) = 0;

  // Get model name
  virtual std::string get_model_name() const = 0;
};

}  // namespace network_sim

#endif  // NETWORK_SIM__COMMUNICATION_MODEL_INTERFACE_HPP_
