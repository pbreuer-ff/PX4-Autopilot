/****************************************************************************
 *
 *   Copyright (c) 2018-2022 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskGimbalFollow.cpp
 */

#include "FlightTaskGimbalFollow.hpp"

using namespace matrix;

bool FlightTaskGimbalFollow::activate(const vehicle_local_position_setpoint_s &last_setpoint)
{
	// all requirements from position-mode still have to hold
	bool ret = FlightTaskManualPosition::activate(last_setpoint);

	_gimbal_rate_setpoint = Vector2f(0.f, 0.f); // ensure gimbal is not moving

	/* _updateHeadingSetpoints();

	gimbal_device_attitude_status_s gimbal_device_attitude_status;

	if (_gimbal_device_attitude_status_sub.update(&gimbal_device_attitude_status)) { // ensure gimbal is pointing in vehicle heading direction

		if (gimbal_device_attitude_status.device_flags & gimbal_device_attitude_status_s::DEVICE_FLAGS_YAW_LOCK) {

			_gimbal_attitude_setpoint = Vector2f(4.f, 0.f);
		}

		else {
			_gimbal_attitude_setpoint = Vector2f(0.f, 0.f);
		}
	}

	_publishGimbalSetpoints(); */

	_gimbal_attitude_setpoint = Vector2f(NAN, NAN); // ensure that no attitude cmds are sent out

	PX4_DEBUG("Gimbal follow task activated: %u", ret);

  	return ret;
}

void FlightTaskGimbalFollow::_scaleSticks()
{
	// Use sticks input with deadzone and exponential curve for vertical velocity
	const float vel_max_z = (_sticks.getPosition()(2) > 0.0f) ? _constraints.speed_down : _constraints.speed_up;
	_velocity_setpoint(2) = vel_max_z * _sticks.getPositionExpo()(2);

	/* Constrain length of stick inputs to 1 for x (roll)*/
	float stick_roll = _sticks.getPositionExpo()(1);

	Vector2f stick_xy_pseudo = {0.0f, stick_roll}; // only roll (zero pitch)

	const float mag = math::constrain(stick_xy_pseudo.length(), 0.0f, 1.0f);

	if (mag > FLT_EPSILON) {
		stick_xy_pseudo = stick_xy_pseudo.normalized() * mag;
	}

	const float max_speed_from_estimator = _sub_vehicle_local_position.get().vxy_max;

	float velocity_scale = _param_mpc_vel_manual.get();

	if (PX4_ISFINITE(max_speed_from_estimator)) {
		// Constrain with optical flow limit but leave 0.3 m/s for repositioning
		velocity_scale = math::constrain(velocity_scale, 0.3f, max_speed_from_estimator);
	}

	// scale velocity to its maximum limits
	Vector2f vel_sp_xy = stick_xy_pseudo * velocity_scale;

	/* Rotate setpoint into local frame. */
	_rotateIntoHeadingFrame(vel_sp_xy);

	_velocity_setpoint.xy() = vel_sp_xy;

	gimbal_device_attitude_status_s gimbal_device_attitude_status;

	if (_gimbal_device_attitude_status_sub.update(&gimbal_device_attitude_status)) {

		Quaternion<float> q_gimbal(gimbal_device_attitude_status.q);
		Euler<float> rpy_gimbal(q_gimbal); // rad
		float yaw_gimbal = rpy_gimbal(2);

		// _sticks.getPosition(Expo) entries in order pitch, roll, throttle, yaw
		float stick_pan = _sticks.getPositionExpo()(3);

		if (gimbal_device_attitude_status.device_flags & gimbal_device_attitude_status_s::DEVICE_FLAGS_YAW_LOCK) {

			PX4_DEBUG("Gimbal yaw lock active, gimbal yaw angle relative to absolute North!");

			if (gimbal_device_attitude_status.angular_velocity_z < 0.01f) _yaw_setpoint = yaw_gimbal; // only adjust heading to gimbal if gimbal no longer panning

			_gimbal_rate_setpoint(0) = stick_pan;
		}
		else { // angles relative to vehicle heading

			PX4_DEBUG("Gimbal yaw lock inactive, gimbal yaw angle relative to aircraft heading!");

			_yawspeed_setpoint = yaw_gimbal;
			_gimbal_rate_setpoint(0) = -_yawspeed_setpoint;

			if (abs(stick_pan) > 0.01f) _gimbal_rate_setpoint(0) = stick_pan; // allow vehicle heading adjustment
		}
	}

	// _gimbal_rate_setpoint(0) = _sticks.getPositionExpo()(3); // pan <-> yaw stick
	_gimbal_rate_setpoint(1) = _sticks.getPositionExpo()(0); // tilt <-> pitch stick
}

void FlightTaskGimbalFollow::_updateSetpoints()
{
	FlightTaskManualAltitude::_updateSetpoints(); // needed to get yaw and setpoints in z-direction
	_acceleration_setpoint.setNaN(); // don't use the horizontal setpoints from FlightTaskManualAltitude

	_updateXYlock(); // check for position lock
	_publishGimbalSetpoints();
}


void FlightTaskGimbalFollow::_publishGimbalSetpoints()
{

	_gimbal_setpoint.timestamp = hrt_absolute_time();
	// _gimbal_setpoint.origin_sysid = 255;
	// _gimbal_setpoint.origin_compid = 0;
	_gimbal_setpoint.target_system = 1;
	_gimbal_setpoint.target_component = 0;
	_gimbal_setpoint.flags = gimbal_manager_set_attitude_s::GIMBAL_MANAGER_FLAGS_YAW_LOCK;
	_gimbal_setpoint.gimbal_device_id = 0; // comp id of gimbal (0 for all gimbals)

	matrix::Eulerf rpy(0.f, _gimbal_attitude_setpoint(1), _gimbal_attitude_setpoint(0));
	matrix::Quatf q(rpy);

	q.copyTo(_gimbal_setpoint.q); // always NAN (no cmd) except during activate()

	_gimbal_setpoint.angular_velocity_x =  0.0f; // no tilt <-> roll authority
	_gimbal_setpoint.angular_velocity_y = _gimbal_rate_setpoint(1); // pitch (tilt)
	_gimbal_setpoint.angular_velocity_z = _gimbal_rate_setpoint(0); // yaw (pan)

	orb_publish(ORB_ID(gimbal_manager_set_attitude), _gimbal_manager_set_attitude_pub, &_gimbal_setpoint);
}
