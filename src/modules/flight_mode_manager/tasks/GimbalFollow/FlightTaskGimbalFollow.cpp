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

  	PX4_INFO("FlightTaskGimbalFollow activate was called! ret: %d", ret); // report if activation was successful

  	return ret;
}

void FlightTaskGimbalFollow::_scaleSticks()

	// Multicopter control
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

	// set _yawspeed_setpoint to adjust to gimbal pan TODO

	/* gimbal_device_attitude_status_s gimbal_device_attitude_status;

	if (_gimbal_device_attitude_status_sub.update(&gimbal_device_attitude_status)) {

		Quaternion<float> q_gimbal(gimbal_device_attitude_status.q);
		Euler<float> ypr_gimbal(q_gimbal);
		float yaw_gimbal = ypr_gimbal(0);

		if (gimbal_device_attitude_status.device_flags & gimbal_device_attitude_status_s::DEVICE_FLAGS_YAW_LOCK) { // angles relative to absolute north

			_yaw_setpoint = yaw_gimbal; // TODO: (pbreuer) is gimbal world frame aligned w/ absolute north too?
		}
		else { // angles relative to vehicle heading

			_yaw_setpoint = yaw_gimbal + _yaw;
		}
	} */

	// Gimbal control

	// TODO: (pbreuer) implement custom scaling for gimbal use here

	// _sticks.getPosition(Expo) entries in order pitch, roll, throttle, yaw
	_gimbal_rate_setpoint(0) = _sticks.getPositionExpo()(3); // pan <-> yaw stick
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

	// gimbal_manager_set_attitude_s gimbal_attitude{};
	gimbal_attitude.timestamp = hrt_absolute_time();
	// gimbal_attitude.origin_sysid = 255; // check!
	// gimbal_attitude.origin_compid = 0; // check!
	gimbal_attitude.target_system = 1;
	gimbal_attitude.target_component = 0;
	gimbal_attitude.flags = 0; // check!
	gimbal_attitude.gimbal_device_id = 0; // comp id of gimbal (0 for all gimbals)

	matrix::Quatf q(NAN, NAN, NAN, NAN);
	q.copyTo(gimbal_attitude.q); // don't command gimbal attitude

	gimbal_attitude.angular_velocity_x =  0.0f; // no tilt <-> roll authority
	gimbal_attitude.angular_velocity_y = _gimbal_rate_setpoint(1); // pitch (tilt)
	gimbal_attitude.angular_velocity_z = _gimbal_rate_setpoint(0); // yaw (pan)

	// PX4_DEBUG("_gimbal_rate_setpoint(2): %lf", static_cast<double>(_gimbal_rate_setpoint(2)));

	//_gimbal_manager_set_attitude_pub.publish(gimbal_attitude);

	orb_publish(ORB_ID(gimbal_manager_set_attitude), _gimbal_manager_set_attitude_pub, &gimbal_attitude);
}
