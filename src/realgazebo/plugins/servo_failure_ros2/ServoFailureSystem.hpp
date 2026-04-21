#ifndef GZ_SIM_SYSTEMS_SERVOFAILUREROS2SYSTEM_HPP_
#define GZ_SIM_SYSTEMS_SERVOFAILUREROS2SYSTEM_HPP_

#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace custom
{

/// \brief ROS2-driven servo failure injection system for Gazebo.
///
/// Subscribes to a Float32MultiArray topic whose payload is laid out as
/// [mode_0, param_0, mode_1, param_1, ...] (one pair per configured servo
/// joint, in SDF declaration order). Runs in PreUpdate AFTER the
/// JointPositionController writes JointForceCmd, then overrides the force
/// (and for LOCK, pins JointVelocityCmd to zero) to simulate the fault.
class ServoFailureROS2System:
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	enum FailureMode : uint8_t {
		NORMAL = 0,
		FLOPPY = 1,  // zero actuation — free-swinging linkage
		LOCK   = 2,  // freeze at current position — stuck gear
		SCALE  = 3,  // reduced control authority — scale force by param
		LIMIT  = 4,  // range-of-motion: param is ratio (0~1) of SDF joint limits
	};

	ServoFailureROS2System();
	~ServoFailureROS2System() override;

	void Configure(const gz::sim::Entity &_entity,
		       const std::shared_ptr<const sdf::Element> &_sdf,
		       gz::sim::EntityComponentManager &_ecm,
		       gz::sim::EventManager &_eventMgr) override;

	void PreUpdate(const gz::sim::UpdateInfo &_info,
		       gz::sim::EntityComponentManager &_ecm) override;

private:
	struct ServoState {
		std::string joint_name;
		gz::sim::Entity joint_entity{0};
		uint8_t mode{NORMAL};
		float param{0.0f};
		double sdf_lower{0.0};  // SDF <axis><limit><lower>
		double sdf_upper{0.0};  // SDF <axis><limit><upper>
		bool   has_limits{false};  // true iff both bounds are finite (|.| < 100 rad)
	};

	void FailureCallback(const std_msgs::msg::Float32MultiArray::SharedPtr _msg);
	void ApplyFaults(gz::sim::EntityComponentManager &_ecm);
	const char *ModeName(uint8_t mode) const;

	rclcpp::Node::SharedPtr ros_node_;
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;

	gz::sim::Model model_;
	std::vector<ServoState> servos_;
	std::string ros_topic_;
	std::mutex mutex_;
};

}  // namespace custom

#endif  // GZ_SIM_SYSTEMS_SERVOFAILUREROS2SYSTEM_HPP_
