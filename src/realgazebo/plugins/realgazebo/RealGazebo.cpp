/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "RealGazebo.hpp"

#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/sim/Util.hh>

using namespace custom;

GZ_ADD_PLUGIN(
	RealGazebo,
	gz::sim::System,
	RealGazebo::ISystemConfigure,
	RealGazebo::ISystemPostUpdate
)

RealGazebo::RealGazebo() :
	entity_id_(0),
	type_code_(255),
	sock_unreal_(-1),
	num_motor_joint_(0),
	num_moveable_link_(0),
	counter_(0)
{
}

RealGazebo::~RealGazebo()
{
	if (sock_unreal_ >= 0) {
		// Send reset message before closing socket
		sendResetMessage();
		close(sock_unreal_);
	}

	// Clean up ROS2 resources safely
	if (ros_node_) {
		try {
			// Reset subscriptions first
			if (battery_status_sub_) {
				battery_status_sub_.reset();
			}
			if (vehicle_status_sub_) {
				vehicle_status_sub_.reset();
			}

			// Then reset the node
			ros_node_.reset();

			gzdbg << "[RealGazebo] Cleaned up ROS2 resources" << std::endl;

		} catch (const std::exception &e) {
			// Ignore exceptions during shutdown - ROS2 may have already shut down
			gzdbg << "[RealGazebo] Exception during cleanup (expected if ROS2 already shutdown): "
			      << e.what() << std::endl;
		}
	}
}

void RealGazebo::Configure(const gz::sim::Entity &_entity,
				      const std::shared_ptr<const sdf::Element> &_sdf,
				      gz::sim::EntityComponentManager &_ecm,
				      gz::sim::EventManager &_eventMgr)
{
	model_entity_ = _entity;
	model_ = gz::sim::Model(model_entity_);
	
	auto model_name_comp = _ecm.Component<gz::sim::components::Name>(model_entity_);
	if (model_name_comp) {
		std::string model_name = model_name_comp->Data();
		
		// Model names follow the '{type}_{id}' convention. A name without a
		// numeric suffix keeps its full name as the type with id 0 (no more
		// silent fallback to a fictional "iris" type).
		size_t underscore_pos = model_name.find_last_of('_');
		std::string suffix = (underscore_pos != std::string::npos)
			? model_name.substr(underscore_pos + 1) : "";
		if (!suffix.empty() &&
		    suffix.find_first_not_of("0123456789") == std::string::npos) {
			entity_type_ = model_name.substr(0, underscore_pos);
			entity_id_ = static_cast<uint8_t>(std::stoi(suffix));
		} else {
			gzwarn << "RealGazebo: model name '" << model_name
			       << "' has no numeric _<id> suffix; using id 0" << std::endl;
			entity_type_ = model_name;
			entity_id_ = 0;
		}
	}
	
	if (_sdf->HasElement("unreal_ip")) {
		unreal_ip_ = _sdf->Get<std::string>("unreal_ip");
	} else {
		unreal_ip_ = "127.0.0.1";
	}
	
	if (_sdf->HasElement("unreal_port")) {
		unreal_port_ = _sdf->Get<int>("unreal_port");
	} else {
		unreal_port_ = 5555;
	}

	// Wire type code: the SDF <type_code> element is the single source of
	// truth (the manager scans the same element). No hardcoded fallback:
	// a model without it fails loudly here and the plugin stays inert,
	// instead of streaming under a silently wrong code.
	if (_sdf->HasElement("type_code")) {
		type_code_ = static_cast<uint8_t>(_sdf->Get<int>("type_code"));
	} else {
		gzerr << "RealGazebo: model '" << entity_type_ << "_"
		      << static_cast<int>(entity_id_)
		      << "' declares no <type_code>; plugin disabled (add "
		      << "<type_code> to the model's RealGazebo plugin block)"
		      << std::endl;
		return;
	}

	setupSendSocket(sock_unreal_, addr_unreal_, unreal_port_);
	
	int idx = 0;
	if (_sdf->HasElement("motorJointList")) {
		auto motor_joint_list = _sdf->FindElement("motorJointList");
		if (motor_joint_list && motor_joint_list->HasElement("motorJoint")) {
			auto motor_joint = motor_joint_list->FindElement("motorJoint");
			
			while (motor_joint && idx < MAX_MOTOR_JOINT) {
				if (motor_joint->HasAttribute("name")) {
					std::string motor_joint_name = motor_joint->Get<std::string>("name");
					auto joint_entity = model_.JointByName(_ecm, motor_joint_name);
					if (joint_entity != gz::sim::kNullEntity) {
						motor_joints_.push_back(joint_entity);
						idx++;
					}
				}
				motor_joint = motor_joint->GetNextElement("motorJoint");
			}
		}
	}
	num_motor_joint_ = idx;
	
	idx = 0;
	if (_sdf->HasElement("moveableLinkList")) {
		auto moveable_link_list = _sdf->FindElement("moveableLinkList");
		if (moveable_link_list && moveable_link_list->HasElement("moveableLink")) {
			auto moveable_link = moveable_link_list->FindElement("moveableLink");
			
			while (moveable_link && idx < MAX_MOVEABLE_LINK) {
				if (moveable_link->HasAttribute("name")) {
					std::string moveable_name = moveable_link->Get<std::string>("name");
					auto link_entity = model_.LinkByName(_ecm, moveable_name);
					if (link_entity != gz::sim::kNullEntity) {
						moveable_links_.push_back(link_entity);
						idx++;
					}
				}
				moveable_link = moveable_link->GetNextElement("moveableLink");
			}
		}
	}
	num_moveable_link_ = idx;
	
	gzmsg << "RealGazebo Model Plugin: Loaded for " << entity_type_ << "_" << static_cast<int>(entity_id_)
	      << " with " << num_motor_joint_ << " motors and " << num_moveable_link_ << " moveable links." << std::endl;

	// Initialize ROS2, if it has not already been initialized
	if (!rclcpp::ok()) {
		int argc = 0;
		char **argv = nullptr;
		rclcpp::init(argc, argv);
	}

	// Create ROS2 node
	std::string model_name_str = entity_type_ + "_" + std::to_string(static_cast<int>(entity_id_));
	ros_node_ = rclcpp::Node::make_shared("realgazebo_" + model_name_str);

	// Get ROS2 topic names for subscriptions
	// ROS namespaces are 1-based, unlike the 0-based entity_id_: /vehicle1
	// belongs to entity_id_ 0.
	std::string vehicle_namespace = "/vehicle" + std::to_string(static_cast<int>(entity_id_) + 1);

	if (_sdf->HasElement("BatteryStatusTopic")) {
		battery_status_topic_ = _sdf->Get<std::string>("BatteryStatusTopic");
	} else {
		battery_status_topic_ = vehicle_namespace + "/fmu/out/battery_status";
	}

	if (_sdf->HasElement("VehicleStatusTopic")) {
		vehicle_status_topic_ = _sdf->Get<std::string>("VehicleStatusTopic");
	} else {
		vehicle_status_topic_ = vehicle_namespace + "/fmu/out/vehicle_status_v1";
	}

	// Create ROS2 subscriptions with sensor_data QoS
	auto qos = rclcpp::SensorDataQoS();

	battery_status_sub_ = ros_node_->create_subscription<px4_msgs::msg::BatteryStatus>(
		battery_status_topic_, qos,
		std::bind(&RealGazebo::BatteryStatusCallback, this, std::placeholders::_1));

	vehicle_status_sub_ = ros_node_->create_subscription<px4_msgs::msg::VehicleStatus>(
		vehicle_status_topic_, qos,
		std::bind(&RealGazebo::VehicleStatusCallback, this, std::placeholders::_1));

	// Send initialization/reset message (data_type = 4)
	sendResetMessage();
}

void RealGazebo::PostUpdate(const gz::sim::UpdateInfo &_info,
				       const gz::sim::EntityComponentManager &_ecm)
{
	// Inert when Configure aborted (e.g. missing <type_code>): no socket,
	// no ROS node, nothing to stream.
	if (sock_unreal_ < 0) {
		return;
	}

	// Spin ROS2 to process callbacks
	if (ros_node_ && rclcpp::ok()) {
		rclcpp::spin_some(ros_node_);
	}

	if (counter_ % 10 == 0) {
		auto world_pose_comp = _ecm.Component<gz::sim::components::WorldPose>(model_entity_);
		gz::math::Pose3d world_pose;
		if (world_pose_comp) {
			world_pose = world_pose_comp->Data();
		} else {
			auto pose_comp = _ecm.Component<gz::sim::components::Pose>(model_entity_);
			if (pose_comp) {
				world_pose = pose_comp->Data();
			}
		}
		
		// 1. Pose data
		auto q = world_pose.Rot();
		const size_t pose_payload_size = sizeof(RealGazeboPacketHeader) + 7 * sizeof(float);
		std::vector<uint8_t> pose_buffer(pose_payload_size);
		
		RealGazeboPacketHeader* pose_header = reinterpret_cast<RealGazeboPacketHeader*>(pose_buffer.data());
		pose_header->entity_id = entity_id_;
		pose_header->type_code = type_code_;
		pose_header->data_type = 1;
		
		float pose_values[7] = {
			static_cast<float>(world_pose.Pos().X()), static_cast<float>(world_pose.Pos().Y()), static_cast<float>(world_pose.Pos().Z()),
			static_cast<float>(q.X()), static_cast<float>(q.Y()), static_cast<float>(q.Z()), static_cast<float>(q.W())
		};
		std::memcpy(pose_buffer.data() + sizeof(RealGazeboPacketHeader), pose_values, sizeof(pose_values));
		
		sendto(sock_unreal_, pose_buffer.data(), pose_payload_size, 0,
		       reinterpret_cast<struct sockaddr*>(&addr_unreal_), sizeof(addr_unreal_));
		
		// 2. RPM data
		if (num_motor_joint_ > 0) {
			const size_t rpm_payload_size = sizeof(RealGazeboPacketHeader) + num_motor_joint_ * sizeof(float);
			std::vector<uint8_t> rpm_buffer(rpm_payload_size);
			
			RealGazeboPacketHeader* rpm_header = reinterpret_cast<RealGazeboPacketHeader*>(rpm_buffer.data());
			rpm_header->entity_id = entity_id_;
			rpm_header->type_code = type_code_;
			rpm_header->data_type = 2;
			
			float* rpm_data_ptr = reinterpret_cast<float*>(rpm_buffer.data() + sizeof(RealGazeboPacketHeader));
			for (int i = 0; i < num_motor_joint_; i++) {
				auto joint_vel = _ecm.Component<gz::sim::components::JointVelocity>(motor_joints_[i]);
				if (joint_vel && !joint_vel->Data().empty()) {
					rpm_data_ptr[i] = static_cast<float>(joint_vel->Data()[0]);
				} else {
					rpm_data_ptr[i] = 0.0f;
				}
			}
			
			sendto(sock_unreal_, rpm_buffer.data(), rpm_payload_size, 0,
			       reinterpret_cast<struct sockaddr*>(&addr_unreal_), sizeof(addr_unreal_));
		}
		
		// 3. Moveable Link data
		if (num_moveable_link_ > 0) {
			const size_t moveable_payload_size = sizeof(RealGazeboPacketHeader) + num_moveable_link_ * 7 * sizeof(float);
			std::vector<uint8_t> moveable_buffer(moveable_payload_size);
			
			RealGazeboPacketHeader* moveable_header = reinterpret_cast<RealGazeboPacketHeader*>(moveable_buffer.data());
			moveable_header->entity_id = entity_id_;
			moveable_header->type_code = type_code_;
			moveable_header->data_type = 3;
			
			float* moveable_data_ptr = reinterpret_cast<float*>(moveable_buffer.data() + sizeof(RealGazeboPacketHeader));
			for (int i = 0; i < num_moveable_link_; i++) {
				auto link_pose_comp = _ecm.Component<gz::sim::components::Pose>(moveable_links_[i]);
				gz::math::Pose3d moveable_pose;
				if (link_pose_comp) {
					moveable_pose = link_pose_comp->Data();
				}
				
				auto moveable_q = moveable_pose.Rot();
				moveable_data_ptr[i * 7 + 0] = static_cast<float>(moveable_pose.Pos().X());
				moveable_data_ptr[i * 7 + 1] = static_cast<float>(moveable_pose.Pos().Y());
				moveable_data_ptr[i * 7 + 2] = static_cast<float>(moveable_pose.Pos().Z());
				moveable_data_ptr[i * 7 + 3] = static_cast<float>(moveable_q.X());
				moveable_data_ptr[i * 7 + 4] = static_cast<float>(moveable_q.Y());
				moveable_data_ptr[i * 7 + 5] = static_cast<float>(moveable_q.Z());
				moveable_data_ptr[i * 7 + 6] = static_cast<float>(moveable_q.W());
			}
			
			sendto(sock_unreal_, moveable_buffer.data(), moveable_payload_size, 0,
			       reinterpret_cast<struct sockaddr*>(&addr_unreal_), sizeof(addr_unreal_));
		}

		// 5. Additional data (battery remaining + nav_state)
		{
			float battery_remaining = 0.0f;
			uint8_t nav_state = 0;

			{
				std::lock_guard<std::mutex> lock(battery_status_mutex_);
				battery_remaining = battery_status_.remaining;
			}

			{
				std::lock_guard<std::mutex> lock(vehicle_status_mutex_);
				nav_state = vehicle_status_.nav_state;
			}

			const size_t additional_payload_size = sizeof(RealGazeboPacketHeader) + sizeof(float) + sizeof(uint8_t);
			std::vector<uint8_t> additional_buffer(additional_payload_size);

			RealGazeboPacketHeader* additional_header = reinterpret_cast<RealGazeboPacketHeader*>(additional_buffer.data());
			additional_header->entity_id = entity_id_;
			additional_header->type_code = type_code_;
			additional_header->data_type = 5;

			std::memcpy(additional_buffer.data() + sizeof(RealGazeboPacketHeader), &battery_remaining, sizeof(float));
			std::memcpy(additional_buffer.data() + sizeof(RealGazeboPacketHeader) + sizeof(float), &nav_state, sizeof(uint8_t));

			sendto(sock_unreal_, additional_buffer.data(), additional_payload_size, 0,
			       reinterpret_cast<struct sockaddr*>(&addr_unreal_), sizeof(addr_unreal_));
		}
	}

	counter_++;
}

void RealGazebo::setupSendSocket(int &sock, struct sockaddr_in &addr, int port)
{
	sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0) {
		gzerr << "Failed to create UDP socket" << std::endl;
		return;
	}

	std::memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);

	// Use getaddrinfo to support both IP addresses and hostnames
	struct addrinfo hints{}, *res;
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_DGRAM;

	if (getaddrinfo(unreal_ip_.c_str(), nullptr, &hints, &res) == 0 && res != nullptr) {
		addr.sin_addr = reinterpret_cast<struct sockaddr_in*>(res->ai_addr)->sin_addr;
		gzmsg << "RealGazebo: Resolved " << unreal_ip_ << " to "
		      << inet_ntoa(addr.sin_addr) << ":" << port << std::endl;
		freeaddrinfo(res);
	} else {
		gzerr << "RealGazebo: Failed to resolve hostname: " << unreal_ip_ << std::endl;
	}
}

void RealGazebo::sendResetMessage()
{
	if (sock_unreal_ < 0) {
		return;
	}

	// Send reset message (data_type = 4, no payload)
	const size_t payload_size = sizeof(RealGazeboPacketHeader);
	std::vector<uint8_t> buffer(payload_size);

	RealGazeboPacketHeader* header = reinterpret_cast<RealGazeboPacketHeader*>(buffer.data());
	header->entity_id = entity_id_;
	header->type_code = type_code_;
	header->data_type = 4;

	sendto(sock_unreal_, buffer.data(), payload_size, 0,
	       reinterpret_cast<struct sockaddr*>(&addr_unreal_), sizeof(addr_unreal_));
}

void RealGazebo::BatteryStatusCallback(const px4_msgs::msg::BatteryStatus::SharedPtr msg)
{
	std::lock_guard<std::mutex> lock(battery_status_mutex_);
	battery_status_ = *msg;
}

void RealGazebo::VehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
	std::lock_guard<std::mutex> lock(vehicle_status_mutex_);
	vehicle_status_ = *msg;
}