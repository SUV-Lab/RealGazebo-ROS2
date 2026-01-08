#ifndef NETWORK_SIM__LOG_DISTANCE_MODEL_HPP_
#define NETWORK_SIM__LOG_DISTANCE_MODEL_HPP_

#include "network_sim/communication_model_interface.hpp"

namespace network_sim
{

class LogDistanceModel : public CommunicationModelInterface {
public:
  // Constructor with parameters
  LogDistanceModel(
    double tx_power_dbm = 20.0,
    double path_loss_exponent = 2.5,
    double reference_distance_m = 1.0,
    double reference_path_loss_db = 40.0,
    double max_retransmission_delay_ms = 50.0,
    double max_jitter_ms = 20.0);

  CommunicationQuality calculate(double distance_m, int vehicles_in_range = 1) override;
  std::string get_model_name() const override;

private:
  // Model parameters
  double tx_power_dbm_;
  double path_loss_exponent_;
  double reference_distance_m_;
  double reference_path_loss_db_;
  double max_retransmission_delay_ms_;
  double max_jitter_ms_;

  // Speed of light (m/s)
  static constexpr double SPEED_OF_LIGHT = 3.0e8;

  // RSSI thresholds for packet loss rate
  static constexpr double RSSI_EXCELLENT = -60.0;  // 0% PLR
  static constexpr double RSSI_POOR = -90.0;       // 100% PLR
};

}  // namespace network_sim

#endif  // NETWORK_SIM__LOG_DISTANCE_MODEL_HPP_
