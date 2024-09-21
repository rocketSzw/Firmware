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

#ifndef CUSTOM_MESSAGE_HPP
#define CUSTOM_MESSAGE_HPP

#include <uORB/topics/custom_message.h>

class MavlinkStreamCustomMessage : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamCustomMessage(mavlink); }

	static constexpr const char *get_name_static() { return "CUSTOM_MESSAGE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_CUSTOM_MESSAGE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _cus_message_sub.advertised() ? MAVLINK_MSG_ID_CUSTOM_MESSAGE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamCustomMessage(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _cus_message_sub{ORB_ID(custom_message)};

	bool send() override
	{

		custom_message_s cus_message;

		if (_cus_message_sub.update(&cus_message)) {
			mavlink_custom_message_t msg{};

			msg.timestamp = cus_message.timestamp / 1000;
			msg.roll_sp = cus_message.roll_sp;
			msg.pitch_sp = cus_message.pitch_sp;
			msg.yaw_sp = cus_message.yaw_sp;
			msg.rollrate_sp = cus_message.rollrate_sp;
			msg.pitchrate_sp = cus_message.pitchrate_sp;
			msg.yawrate_sp = cus_message.yawrate_sp;
			msg.msg1 = cus_message.msg_1;
			msg.msg2 = cus_message.msg_2;
			msg.msg3 = cus_message.msg_3;
			msg.msg4 = cus_message.msg_4;
			msg.msg5 = cus_message.msg_5;
			msg.msg6 = cus_message.msg_6;
			msg.msg7 = cus_message.msg_7;
			msg.msg8 = cus_message.msg_8;
			msg.msg9 = cus_message.msg_9;
			msg.msg10= cus_message.msg_10;

			mavlink_msg_custom_message_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // CUSTOM_MESSAGE_HPP
