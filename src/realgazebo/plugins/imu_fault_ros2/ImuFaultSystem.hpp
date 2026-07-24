#ifndef GZ_SIM_SYSTEMS_IMUFAULTROS2SYSTEM_HPP_
#define GZ_SIM_SYSTEMS_IMUFAULTROS2SYSTEM_HPP_

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <realgazebo_msgs/msg/imu_fault_spec.hpp>

#include "FaultChannelModel.hpp"

namespace custom
{

/// \brief ROS2-driven IMU fault injection system for Gazebo.
///
/// Adds a per-channel fault to the IMU sensor entity's LinearAcceleration and
/// AngularVelocity components in Update, after Physics (priority 0) has filled
/// them and before the stock gz-sim Imu system reads them in PostUpdate. The
/// SDF <sensor> block is left untouched, so the sensor keeps publishing on the
/// topic PX4's gz_bridge hardcodes.
///
/// The failure mode is deliberately open: if this plugin is missing, fails to
/// load, or is disabled, the vehicle gets clean IMU data and behaves exactly as
/// it does without the plugin.
///
/// The required run-after-Physics ordering is declared in C++ via
/// ConfigurePriority(), not in the SDF, so it cannot be lost by an author who
/// omits an XML element. An explicit <gz:system_priority> still overrides it.
class ImuFaultROS2System:
	public gz::sim::System,
	public gz::sim::ISystemConfigurePriority,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemUpdate,
	public gz::sim::ISystemReset
{
public:
	/// \brief Physics runs at kDefaultPriority (0) and fills the IMU
	/// components; this system must run after it to corrupt what the sensor
	/// then reads.
	static constexpr gz::sim::System::PriorityType kAfterPhysics{100};

	gz::sim::System::PriorityType ConfigurePriority() override
	{
		return kAfterPhysics;
	}

	/// \brief Channel order shared by the message, the debug topic and the
	/// applied topic. Accelerometer first, then gyro.
	enum Channel : uint8_t {
		CH_AX = 0,
		CH_AY = 1,
		CH_AZ = 2,
		CH_P  = 3,
		CH_Q  = 4,
		CH_R  = 5,
		CH_COUNT = 6,
	};

	enum Frame : uint8_t {
		FRAME_FRD = 0,
		FRAME_FLU = 1,
	};

	enum TimeRef : uint8_t {
		REF_ON_COMMAND   = 0,
		REF_ON_SPAWN     = 2,
		REF_SIM_TIME_ABS = 3,
	};

	ImuFaultROS2System();
	~ImuFaultROS2System() override;

	void Configure(const gz::sim::Entity &_entity,
		       const std::shared_ptr<const sdf::Element> &_sdf,
		       gz::sim::EntityComponentManager &_ecm,
		       gz::sim::EventManager &_eventMgr) override;

	void Update(const gz::sim::UpdateInfo &_info,
		    gz::sim::EntityComponentManager &_ecm) override;

	void Reset(const gz::sim::UpdateInfo &_info,
		   gz::sim::EntityComponentManager &_ecm) override;

private:
	/// \brief Locate the IMU sensor entity beneath this model. Returns
	/// kNullEntity if it is not in the ECM yet.
	gz::sim::Entity FindImuSensor(gz::sim::EntityComponentManager &_ecm) const;

	/// \brief One-shot report of which components the sensor entity carries.
	/// Confirms the injection contract holds on this gz-sim build.
	void ReportSensorComponents(gz::sim::EntityComponentManager &_ecm);

	void SpecCallback(const realgazebo_msgs::msg::ImuFaultSpec::SharedPtr _msg);
	void DebugCallback(const std_msgs::msg::Float32MultiArray::SharedPtr _msg);

	void PublishApplied(const std::array<double, CH_COUNT> &_fault, double _sim_s);

	rclcpp::Node::SharedPtr ros_node_;
	rclcpp::Subscription<realgazebo_msgs::msg::ImuFaultSpec>::SharedPtr spec_sub_;
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr debug_sub_;
	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr applied_pub_;

	gz::sim::Model model_;
	std::string model_name_;
	std::string sensor_name_{"imu_sensor"};
	gz::sim::Entity sensor_entity_{gz::sim::kNullEntity};

	// --- fault state, guarded by mutex_ ---
	std::mutex mutex_;
	std::array<realgazebo::fault::FaultChannel, CH_COUNT> channels_;
	bool enabled_{false};
	uint8_t frame_{FRAME_FRD};
	uint8_t time_ref_{REF_ON_COMMAND};
	uint32_t spec_seq_{0};
	bool have_spec_{false};
	bool pending_relatch_{false};

	/// \brief Set when the last accepted spec came from the debug topic, which
	/// carries no sequence number. Forces the next real spec to re-latch even if
	/// its spec_seq happens to match the stale counter.
	bool spec_from_debug_{false};

	// --- fault clock ---
	bool t_ref_latched_{false};
	double t_ref_{0.0};

	/// \brief simTime of this plugin instance's first Update tick. Recorded
	/// unconditionally, so a REF_ON_SPAWN spec that arrives later still anchors
	/// to the actual spawn rather than to its own arrival time.
	double spawn_time_{0.0};

	// --- housekeeping ---
	bool first_update_done_{false};
	bool sensor_warned_{false};
	bool components_reported_{false};
	double last_applied_pub_s_{-1.0};
};

}  // namespace custom

#endif  // GZ_SIM_SYSTEMS_IMUFAULTROS2SYSTEM_HPP_
