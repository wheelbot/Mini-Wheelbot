/* dc_motor_impl.hpp
 *
 * Copyright (C) 2019 Christopher Durand
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef LIBMOTOR_DC_MOTOR_HPP
#error "Do not use this file directly, include 'block_commutation.hpp' instead!"
#endif

#include "block_commutation_private.hpp"

namespace librobots2::motor
{

template<typename MotorBridge>
DcMotor<MotorBridge>::DcMotor()
{
	disableMotor();
}

template<typename MotorBridge>
DcMotor<MotorBridge>::~DcMotor()
{
	disableMotor();
}

/// Set pwm value in range of -32767...32767
template<typename MotorBridge>
void DcMotor<MotorBridge>::setSetpoint(int16_t pwm)
{
	constexpr auto Pwm = PhaseConfig::Pwm;
	constexpr auto Low = PhaseConfig::Low;
	constexpr auto HiZ = PhaseConfig::HiZ;

	// limit negative pwm at -32767, 32768 is not representable with int16_t
	if(pwm < -std::numeric_limits<int16_t>::max()) {
		pwm = -std::numeric_limits<int16_t>::max();
	}

	if(pwm == 0) {
		MotorBridge::configure(Low);
		MotorBridge::setCompareValue(0);
	} else if(pwm > 0) {
		MotorBridge::configure({Pwm, Low, HiZ});
		MotorBridge::setCompareValue(pwm / ScaleFactor);
	} else { // pwm < 0
		MotorBridge::configure({Low, Pwm, HiZ});
		MotorBridge::setCompareValue(-pwm / ScaleFactor);
	}
}

template<typename MotorBridge>
void DcMotor<MotorBridge>::disable()
{
	disableMotor();
}

template<typename MotorBridge>
void DcMotor<MotorBridge>::disableMotor()
{
	MotorBridge::configure(PhaseConfig::HiZ);
	MotorBridge::setCompareValue(0);
}

}
