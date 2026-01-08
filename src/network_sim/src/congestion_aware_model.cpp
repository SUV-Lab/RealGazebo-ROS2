#include "network_sim/congestion_aware_model.hpp"

#include <cmath>
#include <algorithm>

namespace network_sim
{

CongestionAwareModel::CongestionAwareModel(
  std::unique_ptr<CommunicationModelInterface> base_model,
  double rssi_threshold_dbm,
  double plr_factor,
  double plr_alpha,
  double latency_beta,
  double jitter_gamma)
: base_model_(std::move(base_model)),
  rssi_threshold_dbm_(rssi_threshold_dbm),
  plr_factor_(plr_factor),
  plr_alpha_(plr_alpha),
  latency_beta_(latency_beta),
  jitter_gamma_(jitter_gamma)
{
}

CommunicationQuality CongestionAwareModel::calculate(double distance_m, int vehicles_in_range)
{
  // Get base quality from wrapped model
  auto quality = base_model_->calculate(distance_m, vehicles_in_range);

  // Apply congestion effects
  apply_congestion_effects(quality, vehicles_in_range);

  return quality;
}

std::string CongestionAwareModel::get_model_name() const
{
  return "Congestion-Aware (" + base_model_->get_model_name() + ")";
}

void CongestionAwareModel::apply_congestion_effects(
  CommunicationQuality& quality,
  int vehicles_in_range)
{
  // No congestion effects with only 1 vehicle
  if (vehicles_in_range <= 1) {
    return;
  }

  // Number of competing vehicles (exclude self)
  int competing_vehicles = vehicles_in_range - 1;

  // 1. Packet Loss Rate: Exponential increase from collision probability
  // PLR_congestion = plr_factor * (1 - exp(-plr_alpha * competing))
  // Based on collision probability in wireless networks
  double plr_congestion = plr_factor_ *
    (1.0 - exp(-plr_alpha_ * competing_vehicles));

  // Add to base packet loss rate, capped at 100%
  quality.packet_loss_rate = std::min(1.0,
    quality.packet_loss_rate + plr_congestion);

  // 2. Latency: Logarithmic increase from CSMA/CA backoff and retransmissions
  // Latency_multiplier = 1 + latency_beta * log(1 + competing)
  // Models exponential backoff delays in MAC layer
  double latency_multiplier = 1.0 +
    latency_beta_ * log(1.0 + competing_vehicles);
  quality.latency_ms *= latency_multiplier;

  // 3. Jitter: Square-root increase from variable delays
  // Jitter_multiplier = 1 + jitter_gamma * sqrt(competing)
  // Models variance in transmission delays
  double jitter_multiplier = 1.0 +
    jitter_gamma_ * sqrt(static_cast<double>(competing_vehicles));
  quality.jitter_ms *= jitter_multiplier;

  // 4. RSSI: Minimal degradation from interference
  // In real wireless networks, congestion primarily causes collisions,
  // not signal strength reduction. Interference effect is minimal (1-3 dB).
  double rssi_degradation = std::min(2.0,
    0.3 * log(1.0 + competing_vehicles));
  quality.rssi_dbm -= rssi_degradation;
}

}  // namespace network_sim
