#ifndef NETWORK_SIM__CONGESTION_AWARE_MODEL_HPP_
#define NETWORK_SIM__CONGESTION_AWARE_MODEL_HPP_

#include "network_sim/communication_model_interface.hpp"
#include <memory>

namespace network_sim
{

class CongestionAwareModel : public CommunicationModelInterface {
public:
  // Constructor with base model and congestion parameters
  CongestionAwareModel(
    std::unique_ptr<CommunicationModelInterface> base_model,
    double rssi_threshold_dbm = -90.0,
    double plr_factor = 0.4,
    double plr_alpha = 0.2,
    double latency_beta = 0.4,
    double jitter_gamma = 0.3);

  CommunicationQuality calculate(double distance_m, int vehicles_in_range = 1) override;
  std::string get_model_name() const override;

private:
  // Wrapped base communication model
  std::unique_ptr<CommunicationModelInterface> base_model_;

  // Congestion model parameters
  double rssi_threshold_dbm_;      // RSSI threshold for "in range"
  double plr_factor_;              // Maximum additional packet loss rate from congestion
  double plr_alpha_;               // Exponential growth rate for packet loss
  double latency_beta_;            // Latency scaling factor
  double jitter_gamma_;            // Jitter scaling factor

  // Apply congestion effects to communication quality
  void apply_congestion_effects(CommunicationQuality& quality, int vehicles_in_range);
};

}  // namespace network_sim

#endif  // NETWORK_SIM__CONGESTION_AWARE_MODEL_HPP_
