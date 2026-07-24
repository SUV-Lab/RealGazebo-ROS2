#include <cmath>

#include <gtest/gtest.h>

#include "FaultChannelModel.hpp"

using realgazebo::fault::EvalFaultChannel;
using realgazebo::fault::FaultChannel;
using realgazebo::fault::EXIT_HOLD_LAST;
using realgazebo::fault::EXIT_SNAP;
using realgazebo::fault::T_BIAS;
using realgazebo::fault::T_DRIFT;
using realgazebo::fault::T_NONE;
using realgazebo::fault::T_OSCILLATION;

namespace
{

constexpr double kDeg = M_PI / 180.0;
constexpr double kEps = 1e-9;

// TABLE II of the reference paper: six channels, window 10 < t < 30.
FaultChannel Ax()
{
	FaultChannel c;
	c.type = T_BIAS;
	c.p0 = 3.0;                 // m/s^2
	c.t_start = 10.0;
	c.t_end = 30.0;
	return c;
}

FaultChannel Ay()
{
	FaultChannel c;
	c.type = T_DRIFT;
	c.p0 = 0.15;                // m/s^2 per s
	c.t_start = 10.0;
	c.t_end = 30.0;
	return c;
}

FaultChannel Az()
{
	FaultChannel c;
	c.type = T_OSCILLATION;
	c.p0 = 3.0;                 // m/s^2
	c.p1 = 0.2;                 // Hz  -> omega = 0.4*pi rad/s
	c.t_start = 10.0;
	c.t_end = 30.0;
	return c;
}

FaultChannel P()
{
	FaultChannel c;
	c.type = T_BIAS;
	c.p0 = 3.0 * kDeg;          // rad/s
	c.t_start = 10.0;
	c.t_end = 30.0;
	return c;
}

FaultChannel Q()
{
	FaultChannel c;
	c.type = T_OSCILLATION;
	c.p0 = 3.0 * kDeg;
	c.p1 = 0.2;
	c.t_start = 10.0;
	c.t_end = 30.0;
	return c;
}

FaultChannel R()
{
	FaultChannel c;
	c.type = T_DRIFT;
	c.p0 = 0.15 * kDeg;
	c.t_start = 10.0;
	c.t_end = 30.0;
	return c;
}

}  // namespace

TEST(FaultChannelModel, InactiveBeforeWindow)
{
	for (const auto &c : {Ax(), Ay(), Az(), P(), Q(), R()}) {
		EXPECT_NEAR(EvalFaultChannel(c, 0.0), 0.0, kEps);
		EXPECT_NEAR(EvalFaultChannel(c, 9.99), 0.0, kEps);
		// The window is open on t_start: 10 < t, not 10 <= t.
		EXPECT_NEAR(EvalFaultChannel(c, 10.0), 0.0, kEps);
	}
}

TEST(FaultChannelModel, BiasIsConstantInsideWindow)
{
	EXPECT_NEAR(EvalFaultChannel(Ax(), 10.01), 3.0, kEps);
	EXPECT_NEAR(EvalFaultChannel(Ax(), 20.0), 3.0, kEps);
	EXPECT_NEAR(EvalFaultChannel(Ax(), 29.99), 3.0, kEps);

	EXPECT_NEAR(EvalFaultChannel(P(), 20.0), 3.0 * kDeg, kEps);
}

TEST(FaultChannelModel, DriftIsLinearFromWindowStart)
{
	// f_Ay = 0.15 * (t - 10)
	EXPECT_NEAR(EvalFaultChannel(Ay(), 10.01), 0.15 * 0.01, kEps);
	EXPECT_NEAR(EvalFaultChannel(Ay(), 20.0), 1.5, kEps);
	EXPECT_NEAR(EvalFaultChannel(Ay(), 29.99), 0.15 * 19.99, kEps);

	// f_r = (0.15 * pi/180) * (t - 10); at t=30- that is 3 deg/s.
	EXPECT_NEAR(EvalFaultChannel(R(), 29.99), 0.15 * kDeg * 19.99, kEps);
}

TEST(FaultChannelModel, OscillationMatchesClosedForm)
{
	// 0.2 Hz -> exactly 4 whole periods across the 20 s window.
	EXPECT_NEAR(EvalFaultChannel(Az(), 10.0 + 1.25), 3.0, kEps);   // quarter period -> peak
	EXPECT_NEAR(EvalFaultChannel(Az(), 10.0 + 2.5), 0.0, kEps);    // half period -> zero
	EXPECT_NEAR(EvalFaultChannel(Az(), 10.0 + 3.75), -3.0, kEps);  // three quarters -> trough
	EXPECT_NEAR(EvalFaultChannel(Az(), 20.0), 0.0, kEps);          // sin(4*pi) == 0

	EXPECT_NEAR(EvalFaultChannel(Q(), 10.0 + 1.25), 3.0 * kDeg, kEps);
}

TEST(FaultChannelModel, HoldLastLatchesAnalyticValueAtWindowEnd)
{
	// Default on_exit is EXIT_SNAP; holding is opt-in.
	ASSERT_EQ(Ax().on_exit, EXIT_SNAP);

	auto hold = [](FaultChannel c) {
		c.on_exit = EXIT_HOLD_LAST;
		return c;
	};

	// Bias holds its magnitude.
	EXPECT_NEAR(EvalFaultChannel(hold(Ax()), 30.0), 3.0, kEps);
	EXPECT_NEAR(EvalFaultChannel(hold(Ax()), 1000.0), 3.0, kEps);

	// Drift holds its terminal value: 0.15 * 20 = 3.0
	EXPECT_NEAR(EvalFaultChannel(hold(Ay()), 30.0), 3.0, kEps);
	EXPECT_NEAR(EvalFaultChannel(hold(Ay()), 1e6), 3.0, kEps);

	// Oscillation holds sin(2*pi*0.2*20) = sin(8*pi) = 0
	EXPECT_NEAR(EvalFaultChannel(hold(Az()), 30.0), 0.0, kEps);
	EXPECT_NEAR(EvalFaultChannel(hold(Az()), 45.0), 0.0, kEps);

	// f_r terminal value is 3 deg/s.
	EXPECT_NEAR(EvalFaultChannel(hold(R()), 30.0), 3.0 * kDeg, kEps);

	// And with the default (snap), TABLE II channels vanish past the window.
	EXPECT_NEAR(EvalFaultChannel(Ax(), 30.01), 0.0, kEps);
	EXPECT_NEAR(EvalFaultChannel(Ay(), 30.01), 0.0, kEps);
}

TEST(FaultChannelModel, HoldLastIsIndependentOfSamplingHistory)
{
	// Stepping right up to the boundary and jumping straight past it must give
	// the same latched value — this is what makes the hold reproducible.
	FaultChannel c = Az();
	c.on_exit = EXIT_HOLD_LAST;
	c.t_end = 25.0;  // 15 s window -> sin(2*pi*0.2*15) = sin(6*pi) = 0
	const double jumped = EvalFaultChannel(c, 100.0);

	double stepped = 0.0;
	for (double tau = 10.0; tau <= 100.0; tau += 0.004) {
		stepped = EvalFaultChannel(c, tau);
	}

	EXPECT_NEAR(jumped, stepped, kEps);
	EXPECT_NEAR(jumped, 0.0, kEps);
}

TEST(FaultChannelModel, SnapDropsToZeroAfterWindow)
{
	FaultChannel c = Ay();
	c.on_exit = EXIT_SNAP;

	EXPECT_NEAR(EvalFaultChannel(c, 29.99), 0.15 * 19.99, kEps);
	EXPECT_NEAR(EvalFaultChannel(c, 30.0), 0.0, kEps);
	EXPECT_NEAR(EvalFaultChannel(c, 30.01), 0.0, kEps);
}

TEST(FaultChannelModel, RampInScalesLinearlyThenSaturates)
{
	FaultChannel c = Ax();
	c.ramp_in = 2.0;

	EXPECT_NEAR(EvalFaultChannel(c, 10.5), 3.0 * 0.25, kEps);
	EXPECT_NEAR(EvalFaultChannel(c, 11.0), 3.0 * 0.5, kEps);
	EXPECT_NEAR(EvalFaultChannel(c, 12.0), 3.0, kEps);
	EXPECT_NEAR(EvalFaultChannel(c, 25.0), 3.0, kEps);
}

TEST(FaultChannelModel, RampLongerThanWindowNeverOvershoots)
{
	// Regression guard: the hold value must respect the fade-in, so a ramp that
	// outlives the window latches the partial value, not the full magnitude.
	FaultChannel c = Ax();
	c.t_end = 15.0;    // 5 s window
	c.ramp_in = 20.0;  // fade-in never completes
	c.on_exit = EXIT_HOLD_LAST;

	EXPECT_NEAR(EvalFaultChannel(c, 30.0), 3.0 * (5.0 / 20.0), kEps);
	EXPECT_LT(EvalFaultChannel(c, 30.0), 3.0);
}

TEST(FaultChannelModel, OpenEndedWindowNeverExits)
{
	FaultChannel c = Ay();
	c.t_end = 0.0;  // <= t_start means "no end"
	c.on_exit = EXIT_SNAP;

	EXPECT_NEAR(EvalFaultChannel(c, 30.0), 0.15 * 20.0, kEps);
	EXPECT_NEAR(EvalFaultChannel(c, 110.0), 0.15 * 100.0, kEps);
}

TEST(FaultChannelModel, NoneTypeIsAlwaysZero)
{
	FaultChannel c;
	c.type = T_NONE;
	c.p0 = 99.0;
	c.t_start = 0.0;
	c.t_end = 100.0;

	EXPECT_NEAR(EvalFaultChannel(c, 50.0), 0.0, kEps);
	EXPECT_NEAR(EvalFaultChannel(c, 200.0), 0.0, kEps);
}
