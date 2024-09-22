/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#ifndef CUSTOM_MAVLINK_INFO_HPP
#define CUSTOM_MAVLINK_INFO_HPP

#include <modules/master_slave/master_slave.h>

#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/custom_transition.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/custom_vel_acc_setpoint.h>
#include <uORB/topics/custom_fw_setpoint.h>

class MavlinkStreamCustomMavlinkInfo : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamCustomMavlinkInfo(mavlink); }

	static constexpr const char *get_name_static() { return "CUSTOM_MAVLINK_INFO"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_CUSTOM_MAVLINK_INFO; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_CUSTOM_MAVLINK_INFO_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamCustomMavlinkInfo(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _mc_virtual_attitude_setpoint_sub{ORB_ID(mc_virtual_attitude_setpoint)};

	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_transition_sub{ORB_ID(custom_transition)};
	uORB::Subscription _vehicle_custom_vel_acc_setpoint_sub{ORB_ID(custom_vel_acc_setpoint)};
	uORB::Subscription _custom_fw_setpoint_sub{ORB_ID(custom_fw_setpoint)};

	mavlink_custom_mavlink_info_t msg{};

	bool send() override
	{
		bool updated = false;

		//sync vel and acc sp, uncomment when use.
		if (_is_sync_type == SYNC_VEL_ACC){
			custom_vel_acc_setpoint_s uavcan_custom_vel_acc_setpoint;
			if (_vehicle_custom_vel_acc_setpoint_sub.update(&uavcan_custom_vel_acc_setpoint)) {
				updated = true;
				msg.mavlink_info[0]=uavcan_custom_vel_acc_setpoint.vel_sp_1;
				msg.mavlink_info[1]=uavcan_custom_vel_acc_setpoint.vel_sp_2;
				msg.mavlink_info[2]=uavcan_custom_vel_acc_setpoint.vel_sp_3;
				msg.mavlink_info[3]=uavcan_custom_vel_acc_setpoint.acc_sp_1;
				msg.mavlink_info[4]=uavcan_custom_vel_acc_setpoint.acc_sp_2;
				msg.mavlink_info[5]=uavcan_custom_vel_acc_setpoint.acc_sp_3;
				msg.mavlink_info[6]=uavcan_custom_vel_acc_setpoint.yaw_sp;
				msg.mavlink_info[7]=uavcan_custom_vel_acc_setpoint.yawspeed_sp;;
				msg.mavlink_info[8]=0;
				msg.mavlink_info[9]=0;
			}
			custom_fw_setpoint_s uavcan_custom_fw_setpoint;
			if (_custom_fw_setpoint_sub.update(&uavcan_custom_fw_setpoint)) {
				updated = true;
				msg.mavlink_info[10] = uavcan_custom_fw_setpoint.fw_sync_roll_sp;
				msg.mavlink_info[11] = uavcan_custom_fw_setpoint.fw_sync_yaw_sp;
				msg.mavlink_info[12] = uavcan_custom_fw_setpoint.fw_sync_altitude;
				msg.mavlink_info[13] = uavcan_custom_fw_setpoint.fw_sync_altitude_sp;
				msg.mavlink_info[14] = uavcan_custom_fw_setpoint.fw_sync_altitude_rate_sp;
				msg.mavlink_info[15] = uavcan_custom_fw_setpoint.fw_sync_altitude_rate_sp_direct;
				msg.mavlink_info[16] = uavcan_custom_fw_setpoint.fw_sync_tas_setpoint;
				msg.mavlink_info[17] = uavcan_custom_fw_setpoint.fw_preserve1;	// fw pitch sp
				msg.mavlink_info[18] = uavcan_custom_fw_setpoint.fw_preserve2;	// fw yawrate sp
			}


			msg.mavlink_info[19]=_is_sync_type_yaw_unlock;


			// sync commands for transition
			custom_transition_s _vehicle_transition;
			if (_vehicle_transition_sub.update(&_vehicle_transition)) {
				updated = true;
				msg.mavlink_info[20] = _vehicle_transition.sync_is_fixed_wing_requested;
				msg.mavlink_info[21] = _vehicle_transition.sync_is_transition_p1_to_p2;
				msg.mavlink_info[22] = _vehicle_transition.sync_exit_backtransition;
			}

			// used to indicate if the master-slave communication is successful
			// monitor the custom_message mavlink message on the slave aircraft, should see q1 changing w.r.t. the attitude from the master aircraft.
			vehicle_attitude_s _vehicle_att;
			if (_vehicle_attitude_sub.update(&_vehicle_att)) {
				updated = true;
				msg.mavlink_info[23] = _vehicle_att.q[1];
			}

			// used to sync the yawspeed command, as on master, this is the final yawspeed command, which could override the command from the attitude control loop
			vehicle_rates_setpoint_s _vehicle_rates_sp;
			if (_vehicle_rates_setpoint_sub.update(&_vehicle_rates_sp)) {
				updated = true;
				msg.mavlink_info[24] = _vehicle_rates_sp.yaw;
			}

			//used to sync when yaw lock
			if (_is_sync_type_yaw_unlock == TRUE){
				vehicle_attitude_setpoint_s _mc_virtual;
				if (_mc_virtual_attitude_setpoint_sub.update(&_mc_virtual)) {
					updated = true;
					msg.mavlink_info[25] = _mc_virtual.roll_body;
					msg.mavlink_info[26] = _mc_virtual.pitch_body;
					msg.mavlink_info[27] = _mc_virtual.yaw_body;
					msg.mavlink_info[28] = _mc_virtual.yaw_sp_move_rate;
					msg.mavlink_info[29] = _mc_virtual.q_d[0];
					msg.mavlink_info[30] = _mc_virtual.q_d[1];
					msg.mavlink_info[31] = _mc_virtual.q_d[2];
					msg.mavlink_info[32] = _mc_virtual.q_d[3];
					msg.mavlink_info[33] = _mc_virtual.reset_integral;
					msg.mavlink_info[34] = _mc_virtual.fw_control_yaw_wheel;
				}
			}
			else {
				msg.mavlink_info[25] = 0;
				msg.mavlink_info[26] = 0;
				msg.mavlink_info[27] = 0;
				msg.mavlink_info[28] = 0;
				msg.mavlink_info[29] = 0;
				msg.mavlink_info[30] = 0;
				msg.mavlink_info[31] = 0;
				msg.mavlink_info[32] = 0;
				msg.mavlink_info[33] = 0;
				msg.mavlink_info[34] = 0;
			}
		}

		if (updated) {
			mavlink_msg_custom_mavlink_info_send_struct(_mavlink->get_channel(), &msg);
		}

		return updated;
	}
};

#endif // CUSTOM_MAVLINK_INFO
