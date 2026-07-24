#include "ImuFaultSystem.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>

using namespace custom;

using gz::math::Vector3d;
using gz::sim::Entity;
using gz::sim::Model;
using gz::sim::kNullEntity;
namespace components = gz::sim::components;

namespace rgfault = realgazebo::fault;

namespace
{

/// \brief Publish the applied-fault telemetry at roughly this rate.
constexpr double kAppliedPubPeriodS = 0.1;

double SimSeconds(const gz::sim::UpdateInfo &_info)
{
	return std::chrono::duration<double>(_info.simTime).count();
}

}  // namespace

//////////////////////////////////////////////////
ImuFaultROS2System::ImuFaultROS2System() = default;

//////////////////////////////////////////////////
ImuFaultROS2System::~ImuFaultROS2System()
{
	if (this->ros_node_) {
		try {
			this->spec_sub_.reset();
			this->debug_sub_.reset();
			this->applied_pub_.reset();
			this->ros_node_.reset();
			gzdbg << "[ImuFaultROS2System] Cleaned up ROS2 resources" << std::endl;

		} catch (const std::exception &e) {
			gzdbg << "[ImuFaultROS2System] Exception during cleanup: "
			      << e.what() << std::endl;
		}
	}
}

//////////////////////////////////////////////////
void ImuFaultROS2System::Configure(const gz::sim::Entity &_entity,
				   const std::shared_ptr<const sdf::Element> &_sdf,
				   gz::sim::EntityComponentManager &_ecm,
				   gz::sim::EventManager & /*_eventMgr*/)
{
	this->model_ = Model(_entity);

	if (!this->model_.Valid(_ecm)) {
		gzerr << "[ImuFaultROS2System] plugin must be attached to a model entity."
		      << std::endl;
		return;
	}

	this->model_name_ = this->model_.Name(_ecm);

	if (_sdf->HasElement("sensorName")) {
		this->sensor_name_ = _sdf->Get<std::string>("sensorName");
	}

	if (_sdf->HasElement("frame")) {
		std::string f = _sdf->Get<std::string>("frame");
		std::transform(f.begin(), f.end(), f.begin(), ::tolower);

		if (f == "flu") {
			this->frame_ = FRAME_FLU;

		} else if (f == "frd") {
			this->frame_ = FRAME_FRD;

		} else {
			gzwarn << "[ImuFaultROS2System] unknown <frame> '" << f
			       << "', falling back to FRD." << std::endl;
		}
	}

	// The sensor entity usually is not in the ECM yet at Configure time, so
	// resolution is retried lazily in Update.
	this->sensor_entity_ = this->FindImuSensor(_ecm);

	if (!rclcpp::ok()) {
		int argc = 0;
		char **argv = nullptr;
		rclcpp::init(argc, argv);
	}

	// Node name must be unique across the vehicles sharing this gzserver.
	std::string node_name = "imu_fault_" + this->model_name_;
	std::replace(node_name.begin(), node_name.end(), '-', '_');
	this->ros_node_ = rclcpp::Node::make_shared(node_name);

	const std::string base = "/" + this->model_name_ + "/imu_fault";

	// Latched so a spec published before this vehicle spawned still arrives —
	// but only while the publishing process stays alive (DDS keeps the sample
	// in the writer, not in the network).
	auto spec_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

	this->spec_sub_ = this->ros_node_->create_subscription<realgazebo_msgs::msg::ImuFaultSpec>(
				  base + "/spec", spec_qos,
				  std::bind(&ImuFaultROS2System::SpecCallback, this, std::placeholders::_1));

	this->debug_sub_ = this->ros_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
				   base + "/commands", 10,
				   std::bind(&ImuFaultROS2System::DebugCallback, this, std::placeholders::_1));

	this->applied_pub_ = this->ros_node_->create_publisher<std_msgs::msg::Float32MultiArray>(
				     base + "/applied", 10);

	gzmsg << "[ImuFaultROS2System] model=" << this->model_name_
	      << " sensor=" << this->sensor_name_
	      << " frame=" << (this->frame_ == FRAME_FRD ? "FRD" : "FLU")
	      << " topics=" << base << "/{spec,commands,applied}"
	      << std::endl;
}

//////////////////////////////////////////////////
Entity ImuFaultROS2System::FindImuSensor(gz::sim::EntityComponentManager &_ecm) const
{
	Entity found = kNullEntity;
	Entity fallback = kNullEntity;

	_ecm.Each<components::Imu, components::Name, components::ParentEntity>(
		[&](const Entity &_entity,
		    const components::Imu *,
		    const components::Name *_name,
		    const components::ParentEntity *_parent) -> bool {
		// The sensor hangs off a link, which hangs off the model.
		auto link_parent = _ecm.Component<components::ParentEntity>(_parent->Data());

		if (!link_parent || link_parent->Data() != this->model_.Entity()) {
			return true;  // belongs to a different model
		}

		if (_name->Data() == this->sensor_name_) {
			found = _entity;
			return false;  // exact match wins, stop scanning
		}

		if (fallback == kNullEntity) {
			fallback = _entity;
		}

		return true;
	});

	if (found != kNullEntity) {
		return found;
	}

	return fallback;
}

//////////////////////////////////////////////////
void ImuFaultROS2System::ReportSensorComponents(gz::sim::EntityComponentManager &_ecm)
{
	if (this->components_reported_ || this->sensor_entity_ == kNullEntity) {
		return;
	}

	this->components_reported_ = true;

	const bool has_accel =
		_ecm.Component<components::LinearAcceleration>(this->sensor_entity_) != nullptr;
	const bool has_gyro =
		_ecm.Component<components::AngularVelocity>(this->sensor_entity_) != nullptr;

	auto name = _ecm.Component<components::Name>(this->sensor_entity_);

	gzmsg << "[ImuFaultROS2System] found sensor entity " << this->sensor_entity_
	      << " (" << (name ? name->Data() : std::string("?")) << ") on model "
	      << this->model_name_
	      << " LinearAcceleration=" << (has_accel ? "yes" : "NO")
	      << " AngularVelocity=" << (has_gyro ? "yes" : "NO")
	      << std::endl;

	if (!has_accel || !has_gyro) {
		gzerr << "[ImuFaultROS2System] sensor entity is missing the components this "
		      << "plugin injects into; fault injection will be a NO-OP. The vehicle "
		      << "keeps flying on clean IMU data." << std::endl;
	}
}

//////////////////////////////////////////////////
void ImuFaultROS2System::Update(const gz::sim::UpdateInfo &_info,
				gz::sim::EntityComponentManager &_ecm)
{
	if (_info.paused) {
		return;
	}

	if (this->ros_node_ && rclcpp::ok()) {
		rclcpp::spin_some(this->ros_node_);
	}

	const double sim_s = SimSeconds(_info);

	// Lazy sensor resolution: the sensor entity appears after Configure.
	if (this->sensor_entity_ == kNullEntity) {
		this->sensor_entity_ = this->FindImuSensor(_ecm);

		if (this->sensor_entity_ == kNullEntity) {
			if (!this->sensor_warned_ && sim_s > 5.0) {
				this->sensor_warned_ = true;
				gzerr << "[ImuFaultROS2System] no IMU sensor named '"
				      << this->sensor_name_ << "' found under model "
				      << this->model_name_
				      << " after 5 s; fault injection disabled." << std::endl;
			}

			return;
		}
	}

	this->ReportSensorComponents(_ecm);

	std::array<double, CH_COUNT> fault{};
	bool any_nonzero = false;
	uint8_t frame = FRAME_FRD;

	{
		std::lock_guard<std::mutex> lock(this->mutex_);

		// Record the spawn instant unconditionally. REF_ON_SPAWN uses the
		// first Update tick rather than Configure, so it is well defined
		// even when the world starts paused — and capturing it regardless
		// of the current time_ref_ means a spec that arrives seconds later
		// can still anchor to the real spawn.
		if (!this->first_update_done_) {
			this->first_update_done_ = true;
			this->spawn_time_ = sim_s;
		}

		if (this->pending_relatch_) {
			this->pending_relatch_ = false;

			switch (this->time_ref_) {
			case REF_SIM_TIME_ABS:
				this->t_ref_ = 0.0;
				break;

			case REF_ON_SPAWN:
				this->t_ref_ = this->spawn_time_;
				break;

			case REF_ON_COMMAND:
			default:
				this->t_ref_ = sim_s;
				break;
			}

			this->t_ref_latched_ = true;
			gzmsg << "[ImuFaultROS2System] " << this->model_name_
			      << " fault clock t_ref = " << this->t_ref_ << " s (seq "
			      << this->spec_seq_ << ")" << std::endl;
		}

		if (this->enabled_ && this->have_spec_ && this->t_ref_latched_) {
			const double tau = sim_s - this->t_ref_;

			for (size_t i = 0; i < CH_COUNT; ++i) {
				fault[i] = rgfault::EvalFaultChannel(this->channels_[i], tau);

				if (fault[i] != 0.0) {
					any_nonzero = true;
				}
			}
		}

		frame = this->frame_;
	}

	this->PublishApplied(fault, sim_s);

	// Nothing to inject: leave the components completely untouched so a
	// disabled plugin is bit-identical to no plugin at all.
	if (!any_nonzero) {
		return;
	}

	Vector3d f_acc(fault[CH_AX], fault[CH_AY], fault[CH_AZ]);
	Vector3d f_gyr(fault[CH_P], fault[CH_Q], fault[CH_R]);

	// The ECM components are in the sensor's FLU frame. PX4's gz_bridge maps
	// FLU -> FRD with (x, y, z) -> (x, -y, -z) (GZBridge.cpp:303), so an
	// FRD-specified fault must be flipped the same way on the way in.
	if (frame == FRAME_FRD) {
		f_acc.Set(f_acc.X(), -f_acc.Y(), -f_acc.Z());
		f_gyr.Set(f_gyr.X(), -f_gyr.Y(), -f_gyr.Z());
	}

	auto accel = _ecm.Component<components::LinearAcceleration>(this->sensor_entity_);

	if (accel) {
		accel->Data() += f_acc;
	}

	auto gyro = _ecm.Component<components::AngularVelocity>(this->sensor_entity_);

	if (gyro) {
		gyro->Data() += f_gyr;
	}
}

//////////////////////////////////////////////////
void ImuFaultROS2System::PublishApplied(const std::array<double, CH_COUNT> &_fault,
					double _sim_s)
{
	if (!this->applied_pub_) {
		return;
	}

	if (this->last_applied_pub_s_ >= 0.0 &&
	    _sim_s - this->last_applied_pub_s_ < kAppliedPubPeriodS) {
		return;
	}

	this->last_applied_pub_s_ = _sim_s;

	std_msgs::msg::Float32MultiArray msg;
	msg.data.resize(CH_COUNT);

	for (size_t i = 0; i < CH_COUNT; ++i) {
		msg.data[i] = static_cast<float>(_fault[i]);
	}

	this->applied_pub_->publish(msg);
}

//////////////////////////////////////////////////
void ImuFaultROS2System::Reset(const gz::sim::UpdateInfo & /*_info*/,
			       gz::sim::EntityComponentManager & /*_ecm*/)
{
	std::lock_guard<std::mutex> lock(this->mutex_);

	this->t_ref_latched_ = false;
	this->t_ref_ = 0.0;
	this->first_update_done_ = false;
	this->last_applied_pub_s_ = -1.0;

	// ON_SPAWN and SIM_TIME_ABS can both re-derive their reference without the
	// operator, so they re-latch on the next tick. ON_COMMAND has no way to
	// know when the fault was "meant" to start, so it stays dormant until a new
	// spec_seq arrives — a reset must never silently restart a fault.
	const bool can_rederive = this->have_spec_ &&
				  (this->time_ref_ == REF_ON_SPAWN ||
				   this->time_ref_ == REF_SIM_TIME_ABS);

	if (can_rederive) {
		this->pending_relatch_ = true;
	}

	gzmsg << "[ImuFaultROS2System] " << this->model_name_
	      << " reset: fault clock cleared"
	      << (this->have_spec_
		  ? (can_rederive ? ", will re-latch next tick"
				  : ", spec left DORMANT until a new spec_seq arrives")
		  : "")
	      << std::endl;
}

//////////////////////////////////////////////////
void ImuFaultROS2System::SpecCallback(
	const realgazebo_msgs::msg::ImuFaultSpec::SharedPtr _msg)
{
	std::lock_guard<std::mutex> lock(this->mutex_);

	if (!_msg->enable) {
		this->enabled_ = false;
		this->have_spec_ = false;
		this->t_ref_latched_ = false;
		this->pending_relatch_ = false;
		this->spec_from_debug_ = false;

		for (auto &c : this->channels_) {
			c = rgfault::FaultChannel{};
		}

		gzmsg << "[ImuFaultROS2System] " << this->model_name_
		      << " fault cleared (enable=false)" << std::endl;
		return;
	}

	// Validate everything into a staging buffer first. A partially applied spec
	// is worse for an experiment than a rejected one: it looks like it worked.
	std::array<rgfault::FaultChannel, CH_COUNT> staged{};

	if (_msg->frame > FRAME_FLU) {
		gzerr << "[ImuFaultROS2System] rejecting spec seq " << _msg->spec_seq
		      << ": unknown frame " << static_cast<int>(_msg->frame) << std::endl;
		return;
	}

	if (_msg->time_ref != REF_ON_COMMAND && _msg->time_ref != REF_ON_SPAWN &&
	    _msg->time_ref != REF_SIM_TIME_ABS) {
		gzerr << "[ImuFaultROS2System] rejecting spec seq " << _msg->spec_seq
		      << ": unknown time_ref " << static_cast<int>(_msg->time_ref)
		      << std::endl;
		return;
	}

	for (const auto &ch : _msg->channels) {
		if (ch.channel >= CH_COUNT) {
			gzerr << "[ImuFaultROS2System] rejecting spec seq " << _msg->spec_seq
			      << ": out-of-range channel " << static_cast<int>(ch.channel)
			      << std::endl;
			return;
		}

		// Codes 4..11 are reserved for non-additive faults this backend
		// cannot express. Accepting them silently would inject nothing while
		// logging success.
		if (ch.type > rgfault::T_OSCILLATION) {
			gzerr << "[ImuFaultROS2System] rejecting spec seq " << _msg->spec_seq
			      << ": fault type " << static_cast<int>(ch.type)
			      << " is not implemented by this plugin" << std::endl;
			return;
		}

		if (ch.on_exit > rgfault::EXIT_HOLD_LAST) {
			gzerr << "[ImuFaultROS2System] rejecting spec seq " << _msg->spec_seq
			      << ": unknown on_exit " << static_cast<int>(ch.on_exit)
			      << std::endl;
			return;
		}

		auto &c = staged[ch.channel];
		c.type = ch.type;
		c.p0 = ch.p0;
		c.p1 = ch.p1;
		c.p2 = ch.p2;
		c.t_start = ch.t_start;
		c.t_end = ch.t_end;
		c.ramp_in = ch.ramp_in;
		c.on_exit = ch.on_exit;
	}

	// Re-latch the clock only when the sequence number actually changes, so a
	// duplicate delivery cannot silently restart the fault mid-experiment. A
	// debug injection carries no sequence, so it always forces a re-latch.
	const bool new_sequence = !this->have_spec_ || this->spec_from_debug_ ||
				  _msg->spec_seq != this->spec_seq_;

	this->channels_ = staged;
	this->frame_ = _msg->frame;
	this->time_ref_ = _msg->time_ref;
	this->spec_seq_ = _msg->spec_seq;
	this->spec_from_debug_ = false;
	this->enabled_ = true;
	this->have_spec_ = true;

	if (new_sequence) {
		this->pending_relatch_ = true;

	} else {
		gzwarn << "[ImuFaultROS2System] " << this->model_name_ << " spec seq "
		       << this->spec_seq_
		       << " repeats the current sequence: parameters updated but the "
		       << "fault clock was NOT restarted. Bump spec_seq to restart it."
		       << std::endl;
	}

	gzmsg << "[ImuFaultROS2System] " << this->model_name_ << " spec seq "
	      << this->spec_seq_ << " accepted (" << _msg->channels.size()
	      << " channels, frame="
	      << (this->frame_ == FRAME_FRD ? "FRD" : "FLU") << ")" << std::endl;
}

//////////////////////////////////////////////////
void ImuFaultROS2System::DebugCallback(
	const std_msgs::msg::Float32MultiArray::SharedPtr _msg)
{
	// Rows of [channel, type, p0, p1, p2, t_start, t_end]. Convenience path for
	// a one-line docker exec smoke test; frame and time_ref cannot be set here.
	constexpr size_t kStride = 7;

	if (_msg->data.empty() || _msg->data.size() % kStride != 0) {
		gzerr << "[ImuFaultROS2System] debug payload length " << _msg->data.size()
		      << " is not a non-zero multiple of " << kStride
		      << "; rejecting the whole message." << std::endl;
		return;
	}

	// Stage and validate before touching any state, so a rejected payload
	// really does leave the running experiment alone.
	const size_t rows = _msg->data.size() / kStride;
	std::array<rgfault::FaultChannel, CH_COUNT> staged{};

	for (size_t i = 0; i < rows; ++i) {
		const float *row = &_msg->data[i * kStride];
		const int ch = static_cast<int>(row[0]);

		if (ch < 0 || ch >= static_cast<int>(CH_COUNT)) {
			gzerr << "[ImuFaultROS2System] debug row " << i
			      << " has out-of-range channel " << ch
			      << "; rejecting the whole message." << std::endl;
			return;
		}

		const int type = static_cast<int>(row[1]);

		if (type < 0 || type > static_cast<int>(rgfault::T_OSCILLATION)) {
			gzerr << "[ImuFaultROS2System] debug row " << i
			      << " has unimplemented fault type " << type
			      << "; rejecting the whole message." << std::endl;
			return;
		}

		auto &c = staged[ch];
		c.type = static_cast<uint8_t>(type);
		c.p0 = row[2];
		c.p1 = row[3];
		c.p2 = row[4];
		c.t_start = row[5];
		c.t_end = row[6];
		c.ramp_in = 0.0;
		c.on_exit = rgfault::EXIT_SNAP;
	}

	std::lock_guard<std::mutex> lock(this->mutex_);

	// The debug path has a fixed contract: FRD, on_command, hold_last. Forcing
	// the frame matters because the README's sign check runs through here, and
	// a leftover FLU from an earlier spec would invert that check's meaning.
	this->channels_ = staged;
	this->enabled_ = true;
	this->have_spec_ = true;
	this->frame_ = FRAME_FRD;
	this->time_ref_ = REF_ON_COMMAND;
	// Deliberately does NOT touch spec_seq_: that counter belongs to the
	// external /spec interface, and colliding with it would make a later real
	// spec look like a duplicate.
	this->spec_from_debug_ = true;
	this->pending_relatch_ = true;

	gzmsg << "[ImuFaultROS2System] " << this->model_name_ << " debug spec accepted ("
	      << rows << " rows, frame=FRD, time_ref=on_command)" << std::endl;
}

GZ_ADD_PLUGIN(
	ImuFaultROS2System,
	gz::sim::System,
	ImuFaultROS2System::ISystemConfigurePriority,
	ImuFaultROS2System::ISystemConfigure,
	ImuFaultROS2System::ISystemUpdate,
	ImuFaultROS2System::ISystemReset
)
