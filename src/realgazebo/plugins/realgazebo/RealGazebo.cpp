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
	vehicle_num_(0),
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
		
		size_t underscore_pos = model_name.find_last_of('_');
		if (underscore_pos != std::string::npos) {
			vehicle_type_ = model_name.substr(0, underscore_pos);
			vehicle_num_ = static_cast<uint8_t>(std::stoi(model_name.substr(underscore_pos + 1)));
		} else {
			vehicle_type_ = "iris";
			vehicle_num_ = 0;
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
	
	gzmsg << "RealGazebo Model Plugin: Loaded for " << vehicle_type_ << "_" << static_cast<int>(vehicle_num_)
	      << " with " << num_motor_joint_ << " motors and " << num_moveable_link_ << " moveable links." << std::endl;

	// Send initialization/reset message (data_type = 4)
	sendResetMessage();
}

void RealGazebo::PostUpdate(const gz::sim::UpdateInfo &_info,
				       const gz::sim::EntityComponentManager &_ecm)
{
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
		pose_header->vehicle_num = vehicle_num_;
		pose_header->vehicle_code = getVehicleCode(vehicle_type_);
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
			rpm_header->vehicle_num = vehicle_num_;
			rpm_header->vehicle_code = getVehicleCode(vehicle_type_);
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
			moveable_header->vehicle_num = vehicle_num_;
			moveable_header->vehicle_code = getVehicleCode(vehicle_type_);
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
	}
	
	counter_++;
}

uint8_t RealGazebo::getVehicleCode(const std::string &vehicle_type) const
{
	if (vehicle_type == "x500") return 0;
	else if (vehicle_type == "rover_ackermann") return 1;
	else if (vehicle_type == "boat") return 2;
	else if (vehicle_type == "lc_62") return 3;
	else if (vehicle_type == "ugv_kimm") return 4;
	else if (vehicle_type == "rock") return 201;
	else return 255;
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
	inet_pton(AF_INET, unreal_ip_.c_str(), &addr.sin_addr);
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
	header->vehicle_num = vehicle_num_;
	header->vehicle_code = getVehicleCode(vehicle_type_);
	header->data_type = 4;

	sendto(sock_unreal_, buffer.data(), payload_size, 0,
	       reinterpret_cast<struct sockaddr*>(&addr_unreal_), sizeof(addr_unreal_));
}