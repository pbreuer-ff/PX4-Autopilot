/****************************************************************************
 *
 *   Copyright (c) 2017-2022 PX4 Development Team. All rights reserved.
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

/**
 * @file FlightTaskGimbalFollow.hpp
 *
 * Flight task for manual gimbal pan and tilt control (vehicle roll and throttle same as FlightTaskManualPosition)
 *
 * @author Peter Breuer <peter@freeflysystems.com>
 */

#pragma once

#include "FlightTaskManualPosition.hpp"
#include <uORB/Publication.hpp>
#include <uORB/topics/gimbal_manager_set_attitude.h>
#include <uORB/topics/gimbal_device_attitude_status.h>

class FlightTaskGimbalFollow : public FlightTaskManualPosition
{
public:
	FlightTaskGimbalFollow() = default;

	virtual ~FlightTaskGimbalFollow() = default;
	bool activate(const vehicle_local_position_setpoint_s &last_setpoint) override;

protected:
	void _updateSetpoints() override;
	void _scaleSticks() override;
	void _publishGimbalSetpoints();

private:

	matrix::Vector2f _gimbal_rate_setpoint; // order: tilt-pan aka pitch-yaw rate
	matrix::Vector2f _gimbal_attitude_setpoint; // order: tilt-pan aka pitch-yaw
	gimbal_manager_set_attitude_s _gimbal_setpoint{};
	orb_advert_t _gimbal_manager_set_attitude_pub = orb_advertise(ORB_ID(gimbal_manager_set_attitude), &_gimbal_setpoint);
	uORB::Subscription _gimbal_device_attitude_status_sub{ORB_ID(gimbal_device_attitude_status)};
};
