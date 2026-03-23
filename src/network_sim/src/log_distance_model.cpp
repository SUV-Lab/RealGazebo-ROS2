#include "network_sim/log_distance_model.hpp"

#include <cmath>

namespace network_sim
{

LogDistanceModel::LogDistanceModel(
  double tx_power_dbm,
  double path_loss_exponent,
  double reference_distance_m,
  double reference_path_loss_db,
  double max_retransmission_delay_ms,
  double max_jitter_ms,
  double baseline_latency_ms,
  double baseline_jitter_ms)
: tx_power_dbm_(tx_power_dbm),
  path_loss_exponent_(path_loss_exponent),
  reference_distance_m_(reference_distance_m),
  reference_path_loss_db_(reference_path_loss_db),
  max_retransmission_delay_ms_(max_retransmission_delay_ms),
  max_jitter_ms_(max_jitter_ms),
  baseline_latency_ms_(baseline_latency_ms),
  baseline_jitter_ms_(baseline_jitter_ms)
{
}

CommunicationQuality LogDistanceModel::calculate(double distance_m, int vehicles_in_range)
{
  CommunicationQuality quality;
  quality.distance_m = distance_m;
  quality.vehicles_in_range = vehicles_in_range;

  // 1. Calculate path loss: PL(d) = PL(d0) + 10*n*log10(d/d0)
  quality.path_loss_db = reference_path_loss_db_ +
    10.0 * path_loss_exponent_ * log10(distance_m / reference_distance_m_);

  // 2. Calculate RSSI: RSSI = TxPower - PathLoss
  quality.rssi_dbm = tx_power_dbm_ - quality.path_loss_db;

  // 3. Calculate packet loss rate (linear interpolation between thresholds)
  if (quality.rssi_dbm >= RSSI_EXCELLENT) {
    quality.packet_loss_rate = 0.0;
  } else if (quality.rssi_dbm <= RSSI_POOR) {
    quality.packet_loss_rate = 1.0;
  } else {
    quality.packet_loss_rate =
      (RSSI_EXCELLENT - quality.rssi_dbm) / (RSSI_EXCELLENT - RSSI_POOR);
  }

  // 4. Calculate latency (baseline + propagation + retransmission)
  double propagation_delay_ms = (distance_m / SPEED_OF_LIGHT) * 1000.0;
  double retransmission_delay_ms =
    quality.packet_loss_rate * max_retransmission_delay_ms_;
  quality.latency_ms = baseline_latency_ms_ + propagation_delay_ms + retransmission_delay_ms;

  // 5. Calculate jitter (baseline + proportional to packet loss rate)
  quality.jitter_ms = baseline_jitter_ms_ + quality.packet_loss_rate * max_jitter_ms_;

  return quality;
}

std::string LogDistanceModel::get_model_name() const
{
  return "Log-Distance Path Loss Model";
}

}  // namespace network_sim
