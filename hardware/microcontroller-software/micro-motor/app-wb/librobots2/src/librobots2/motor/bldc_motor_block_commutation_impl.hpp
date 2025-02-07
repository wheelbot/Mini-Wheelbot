/* dc_motor_impl.hpp
 *
 * Copyright (C) 2019 Christopher Durand
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef LIBMOTOR_BLDC_MOTOR_BLOCK_COMMUTATION_HPP
#error "Do not use this file directly, include 'block_commutation.hpp' instead!"
#endif

#include "block_commutation.hpp"

namespace librobots2::motor
{

template<typename MotorBridge>
BldcMotorBlockCommutation<MotorBridge>::BldcMotorBlockCommutation(uint_fast8_t commutationOffset)
	: commutationOffset_{commutationOffset}
{
	disableMotor();
}

template<typename MotorBridge>
BldcMotorBlockCommutation<MotorBridge>::~BldcMotorBlockCommutation()
{
	disableMotor();
}

template<typename MotorBridge>
void BldcMotorBlockCommutation<MotorBridge>::setSetpoint(int16_t pwm)
{
	// limit negative pwm at -32767, 32768 is not representable with int16_t
	if(pwm < -std::numeric_limits<int16_t>::max()) {
		pwm = -std::numeric_limits<int16_t>::max();
	}

	if (pwm > 0) {
		mode_ = Mode::Pwm;
		reverse_ = false;
	} else if (pwm < 0) {
		mode_ = Mode::Pwm;
		reverse_ = true;
		pwm = -pwm;
	} else /* (pwm == 0) */ {
		mode_ = Mode::Brake;
	}

	MotorBridge::setCompareValue(pwm / ScaleFactor);
	update();
}

template<typename MotorBridge>
void BldcMotorBlockCommutation<MotorBridge>::update()
{
	using Commutation = BlockCommutation<typename MotorBridge::HallPort>;

	BridgeConfig config = {};
	if(mode_ == Mode::Pwm) {
		config = Commutation::doCommutation(commutationOffset_, reverse_);
	} else if(mode_ == Mode::Brake) {
		config = Commutation::motorBrake();
	} else /* disable motor */ {
		config = Commutation::disable();
	}

	MotorBridge::configure(config);
}

template<typename MotorBridge>
void BldcMotorBlockCommutation<MotorBridge>::disable()
{
	disableMotor();
}

template<typename MotorBridge>
void BldcMotorBlockCommutation<MotorBridge>::disableMotor()
{
	mode_ = Mode::Disabled;
	update();
}

// copy/move-constructors, assignment operators. Cannot be defaulted due to volatile members.

template<typename MotorBridge>
BldcMotorBlockCommutation<MotorBridge>::BldcMotorBlockCommutation(const BldcMotorBlockCommutation& other)
	: commutationOffset_{other.commutationOffset_}, reverse_{other.reverse_}, mode_{other.mode_}
{
}

template<typename MotorBridge>
BldcMotorBlockCommutation<MotorBridge>&
BldcMotorBlockCommutation<MotorBridge>::operator=(const BldcMotorBlockCommutation& other)
{
	commutationOffset_ = other.commutationOffset_;
	reverse_ = other.reverse_;
	mode_ = other.mode_;
	return *this;
}

}
