#include "ServoFailureSystem.hpp"

#include <algorithm>
#include <cmath>

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocityCmd.hh>

using namespace custom;

using gz::sim::Entity;
using gz::sim::Model;
using gz::sim::kNullEntity;
namespace components = gz::sim::components;

//////////////////////////////////////////////////
ServoFailureROS2System::ServoFailureROS2System() = default;

//////////////////////////////////////////////////
ServoFailureROS2System::~ServoFailureROS2System()
{
	if (this->ros_node_) {
		try {
			if (this->sub_) {
				this->sub_.reset();
			}

			this->ros_node_.reset();
			gzdbg << "[ServoFailureROS2System] Cleaned up ROS2 resources" << std::endl;

		} catch (const std::exception &e) {
			gzdbg << "[ServoFailureROS2System] Exception during cleanup: "
			      << e.what() << std::endl;
		}
	}
}

//////////////////////////////////////////////////
void ServoFailureROS2System::Configure(const gz::sim::Entity &_entity,
				       const std::shared_ptr<const sdf::Element> &_sdf,
				       gz::sim::EntityComponentManager &_ecm,
				       gz::sim::EventManager & /*_eventMgr*/)
{
	this->model_ = Model(_entity);

	if (!this->model_.Valid(_ecm)) {
		gzerr << "[ServoFailureROS2System] plugin must be attached to a model entity."
		      << std::endl;
		return;
	}

	std::string model_name = this->model_.Name(_ecm);

	if (_sdf->HasElement("ServoFailureTopic")) {
		this->ros_topic_ = _sdf->Get<std::string>("ServoFailureTopic");

	} else {
		this->ros_topic_ = "/" + model_name + "/servo_failure/commands";
	}

	if (_sdf->HasElement("servoJointList")) {
		auto list = _sdf->FindElement("servoJointList");

		if (list && list->HasElement("servoJoint")) {
			auto elem = list->FindElement("servoJoint");

			while (elem) {
				if (elem->HasAttribute("name")) {
					std::string joint_name = elem->Get<std::string>("name");
					auto joint_entity = this->model_.JointByName(_ecm, joint_name);

					if (joint_entity != kNullEntity) {
						ServoState state;
						state.joint_name = joint_name;
						state.joint_entity = joint_entity;

						auto axis = _ecm.Component<components::JointAxis>(joint_entity);

						if (axis) {
							state.sdf_lower = axis->Data().Lower();
							state.sdf_upper = axis->Data().Upper();

							// SDF often uses 1e+16 to mean "no limit"; treat
							// anything absurdly large as unbounded.
							const double kFiniteThreshold = 100.0;  // rad

							if (std::fabs(state.sdf_lower) < kFiniteThreshold &&
							    std::fabs(state.sdf_upper) < kFiniteThreshold) {
								state.has_limits = true;
							}
						}

						if (!state.has_limits) {
							gzwarn << "[ServoFailureROS2System] joint "
							       << joint_name
							       << " has no usable SDF <limit>; LIMIT mode will be a no-op."
							       << std::endl;
						}

						this->servos_.push_back(state);
						gzmsg << "[ServoFailureROS2System] servo["
						      << this->servos_.size() - 1 << "] = "
						      << joint_name
						      << " limits=[" << state.sdf_lower
						      << ", " << state.sdf_upper << "]"
						      << (state.has_limits ? "" : " (unbounded)")
						      << std::endl;

					} else {
						gzwarn << "[ServoFailureROS2System] joint not found: "
						       << joint_name << std::endl;
					}
				}

				elem = elem->GetNextElement("servoJoint");
			}
		}
	}

	if (this->servos_.empty()) {
		gzwarn << "[ServoFailureROS2System] no servo joints configured for model "
		       << model_name << std::endl;
	}

	if (!rclcpp::ok()) {
		int argc = 0;
		char **argv = nullptr;
		rclcpp::init(argc, argv);
	}

	this->ros_node_ = rclcpp::Node::make_shared("servo_failure");
	this->sub_ = this->ros_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
			     this->ros_topic_, 10,
			     std::bind(&ServoFailureROS2System::FailureCallback, this, std::placeholders::_1));

	gzmsg << "[ServoFailureROS2System] subscribed to " << this->ros_topic_
	      << " (" << this->servos_.size() << " servos)" << std::endl;
}

//////////////////////////////////////////////////
void ServoFailureROS2System::ApplyFaults(gz::sim::EntityComponentManager &_ecm)
{
	std::lock_guard<std::mutex> lock(this->mutex_);

	for (auto &s : this->servos_) {
		if (s.joint_entity == kNullEntity) {
			continue;
		}

		if (s.mode == NORMAL) {
			continue;
		}

		auto forceCmd = _ecm.Component<components::JointForceCmd>(s.joint_entity);

		switch (s.mode) {
			case FLOPPY: {
				if (forceCmd) {
					*forceCmd = components::JointForceCmd({0.0});

				} else {
					_ecm.CreateComponent(s.joint_entity,
							     components::JointForceCmd({0.0}));
				}

				break;
			}

			case LOCK: {
				if (forceCmd) {
					*forceCmd = components::JointForceCmd({0.0});

				} else {
					_ecm.CreateComponent(s.joint_entity,
							     components::JointForceCmd({0.0}));
				}

				// Kinematic freeze: physics will enforce velocity = 0 this step.
				auto velCmd = _ecm.Component<components::JointVelocityCmd>(s.joint_entity);

				if (velCmd) {
					*velCmd = components::JointVelocityCmd({0.0});

				} else {
					_ecm.CreateComponent(s.joint_entity,
							     components::JointVelocityCmd({0.0}));
				}

				break;
			}

			case SCALE: {
				if (forceCmd && !forceCmd->Data().empty()) {
					double scale = std::clamp(static_cast<double>(s.param), 0.0, 1.0);
					double scaled = forceCmd->Data()[0] * scale;
					*forceCmd = components::JointForceCmd({scaled});
				}

				break;
			}

			case LIMIT: {
				if (!s.has_limits) {
					break;  // joint has no SDF range to scale; already warned in Configure
				}

				double ratio = std::clamp(static_cast<double>(s.param), 0.0, 1.0);

				// Preserve SDF signs: lower is negative, upper is positive
				// for a typical control-surface joint.
				double eff_lower = s.sdf_lower * ratio;
				double eff_upper = s.sdf_upper * ratio;

				auto posComp = _ecm.Component<components::JointPosition>(s.joint_entity);

				if (!posComp || posComp->Data().empty()) {
					break;  // physics hasn't populated position yet
				}

				double pos = posComp->Data()[0];

				bool beyond_upper = pos > eff_upper;
				bool beyond_lower = pos < eff_lower;

				if (!beyond_upper && !beyond_lower) {
					break;  // within bounds — passthrough
				}

				double force = (forceCmd && !forceCmd->Data().empty()) ? forceCmd->Data()[0] : 0.0;
				bool force_outward = (beyond_upper && force > 0.0) ||
						     (beyond_lower && force < 0.0);

				if (force_outward) {
					// One-way kinematic block: prevent further outward motion.
					if (forceCmd) {
						*forceCmd = components::JointForceCmd({0.0});

					} else {
						_ecm.CreateComponent(s.joint_entity,
								     components::JointForceCmd({0.0}));
					}

					auto velCmd = _ecm.Component<components::JointVelocityCmd>(s.joint_entity);

					if (velCmd) {
						*velCmd = components::JointVelocityCmd({0.0});

					} else {
						_ecm.CreateComponent(s.joint_entity,
								     components::JointVelocityCmd({0.0}));
					}
				}

				// else: JPC force is inward or zero — let it pass through so the
				// joint can travel back into the allowed range.
				break;
			}

			default:
				break;
		}
	}
}

//////////////////////////////////////////////////
void ServoFailureROS2System::PreUpdate(const gz::sim::UpdateInfo &_info,
				       gz::sim::EntityComponentManager &_ecm)
{
	if (_info.paused) {
		return;
	}

	if (this->ros_node_ && rclcpp::ok()) {
		rclcpp::spin_some(this->ros_node_);
	}

	this->ApplyFaults(_ecm);
}

//////////////////////////////////////////////////
void ServoFailureROS2System::FailureCallback(
	const std_msgs::msg::Float32MultiArray::SharedPtr _msg)
{
	std::lock_guard<std::mutex> lock(this->mutex_);

	const auto &data = _msg->data;

	if (data.size() % 2 != 0) {
		gzwarn << "[ServoFailureROS2System] payload length " << data.size()
		       << " is not even; expected [mode, param] pairs." << std::endl;
		return;
	}

	size_t pairs = data.size() / 2;

	for (size_t i = 0; i < this->servos_.size(); ++i) {
		uint8_t new_mode = NORMAL;
		float new_param = 0.0f;

		if (i < pairs) {
			float raw_mode = data[2 * i];
			new_param = data[2 * i + 1];

			if (raw_mode >= 0.0f && raw_mode <= static_cast<float>(LIMIT) + 0.5f) {
				new_mode = static_cast<uint8_t>(raw_mode);

			} else {
				gzwarn << "[ServoFailureROS2System] servo " << i
				       << " invalid mode " << raw_mode
				       << ", forcing NORMAL." << std::endl;
			}
		}

		auto &s = this->servos_[i];

		if (new_mode != s.mode) {
			gzerr << "[ServoFailureROS2System] servo " << i << " ("
			      << s.joint_name << ") " << ModeName(s.mode) << " -> "
			      << ModeName(new_mode) << " param=" << new_param << std::endl;
		}

		s.mode = new_mode;
		s.param = new_param;
	}
}

//////////////////////////////////////////////////
const char *ServoFailureROS2System::ModeName(uint8_t mode) const
{
	switch (mode) {
		case NORMAL: return "NORMAL";

		case FLOPPY: return "FLOPPY";

		case LOCK: return "LOCK";

		case SCALE: return "SCALE";

		case LIMIT: return "LIMIT";

		default: return "UNKNOWN";
	}
}

GZ_ADD_PLUGIN(
	ServoFailureROS2System,
	gz::sim::System,
	ServoFailureROS2System::ISystemConfigure,
	ServoFailureROS2System::ISystemPreUpdate
)
