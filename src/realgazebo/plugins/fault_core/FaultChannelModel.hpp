#ifndef REALGAZEBO_FAULT_CORE_FAULTCHANNELMODEL_HPP_
#define REALGAZEBO_FAULT_CORE_FAULTCHANNELMODEL_HPP_

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace realgazebo
{
namespace fault
{

/// \brief Fault waveform kinds. Values mirror realgazebo_msgs/FaultChannel.
enum FaultType : uint8_t {
	T_NONE        = 0,
	T_BIAS        = 1,
	T_DRIFT       = 2,
	T_OSCILLATION = 3,
};

/// \brief What happens once the activation window closes.
enum ExitMode : uint8_t {
	EXIT_SNAP      = 0,  ///< drop to zero instantly at t_end (default)
	EXIT_HOLD_LAST = 1,  ///< latch the t_end value forever
};

/// \brief Fault description for a single scalar sensor channel.
///
/// Deliberately free of ROS and Gazebo types so it can be unit-tested without a
/// simulator, and reused by a different injection backend (see the relay
/// fallback in docs/imu_fault_injection-plan.md section 8).
struct FaultChannel {
	uint8_t type{T_NONE};

	double p0{0.0};  ///< BIAS: bias | DRIFT: rate [unit/s] | OSC: amplitude
	double p1{0.0};  ///< OSC: frequency [Hz]
	double p2{0.0};  ///< OSC: phase [rad]

	double t_start{0.0};  ///< window open  [s], relative to the fault clock
	double t_end{0.0};    ///< window close [s]; <= t_start means "no end"
	double ramp_in{0.0};  ///< linear fade-in duration [s]; 0 = hard step

	uint8_t on_exit{EXIT_SNAP};
};

/// \brief Waveform value at elapsed time \p d seconds into the window,
///        ignoring the window bounds and the fade-in.
inline double FaultWaveform(const FaultChannel &c, double d)
{
	switch (c.type) {
	case T_BIAS:
		return c.p0;

	case T_DRIFT:
		return c.p0 * d;

	case T_OSCILLATION:
		return c.p0 * std::sin(2.0 * M_PI * c.p1 * d + c.p2);

	default:
		return 0.0;
	}
}

/// \brief Fade-in multiplier at elapsed time \p d seconds into the window.
inline double FaultRampScale(const FaultChannel &c, double d)
{
	if (c.ramp_in <= 0.0) {
		return 1.0;
	}

	return std::clamp(d / c.ramp_in, 0.0, 1.0);
}

/// \brief Fault contribution for this channel at fault-clock time \p tau.
///
/// Pure function of (channel, tau) — carries no state across calls. That makes
/// the result independent of step size, dropped steps and world resets, and
/// removes any need to guard mutable fault state with a mutex.
///
/// EXIT_HOLD_LAST latches the analytic value the waveform HAD at t_end (fade-in
/// included), not whatever the last executed tick happened to sample. For an
/// oscillation that means a deterministic, reproducible hold value rather than
/// one that depends on where the step boundary landed.
inline double EvalFaultChannel(const FaultChannel &c, double tau)
{
	if (c.type == T_NONE) {
		return 0.0;
	}

	const bool has_end = c.t_end > c.t_start;

	// Before the window opens.
	if (tau <= c.t_start) {
		return 0.0;
	}

	// Inside the window (or the window never closes).
	if (!has_end || tau < c.t_end) {
		const double d = tau - c.t_start;
		return FaultRampScale(c, d) * FaultWaveform(c, d);
	}

	// Past the window.
	if (c.on_exit == EXIT_HOLD_LAST) {
		const double d_end = c.t_end - c.t_start;
		return FaultRampScale(c, d_end) * FaultWaveform(c, d_end);
	}

	return 0.0;
}

}  // namespace fault
}  // namespace realgazebo

#endif  // REALGAZEBO_FAULT_CORE_FAULTCHANNELMODEL_HPP_
