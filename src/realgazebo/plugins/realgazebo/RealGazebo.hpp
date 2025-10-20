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

/**
 * @brief RealGazebo Model Plugin
 *
 * This Model plugin sends data for gz-realgazebo
 * Compatible with the legacy RealGazebo protocol for pose, motor RPM, and servo data
 *
 * @author MinKyu Kim <kmk6061602@cbnu.ac.kr>
 */

#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/ParentEntity.hh>

#include <vector>
#include <string>
#include <memory>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

#pragma pack(push, 1)
struct RealGazeboPacketHeader {
  uint8_t vehicle_num;
  uint8_t vehicle_code;
  uint8_t data_type;
};
#pragma pack(pop)

#define MAX_MOTOR_JOINT 16
#define MAX_MOVEABLE_LINK 16

namespace custom
{
class RealGazebo:
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPostUpdate
{
public:
	RealGazebo();
	~RealGazebo();

	void Configure(const gz::sim::Entity &_entity,
		       const std::shared_ptr<const sdf::Element> &_sdf,
		       gz::sim::EntityComponentManager &_ecm,
		       gz::sim::EventManager &_eventMgr) override;

	void PostUpdate(const gz::sim::UpdateInfo &_info,
			const gz::sim::EntityComponentManager &_ecm) override;

private:
	uint8_t getVehicleCode(const std::string &vehicle_type) const;

	void setupSendSocket(int &sock, struct sockaddr_in &addr, int port);

	void sendResetMessage();
	
	gz::sim::Entity model_entity_;
	gz::sim::Model model_;
	
	std::string vehicle_type_;
	uint8_t vehicle_num_;
	
	std::string unreal_ip_;
	int unreal_port_;
	
	int sock_unreal_;
	struct sockaddr_in addr_unreal_;
	
	std::vector<gz::sim::Entity> motor_joints_;
	std::vector<gz::sim::Entity> moveable_links_;
	int num_motor_joint_;
	int num_moveable_link_;
	
	uint64_t counter_;
};
} // end namespace custom
