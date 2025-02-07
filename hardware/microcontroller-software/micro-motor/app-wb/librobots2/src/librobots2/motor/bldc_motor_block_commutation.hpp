/* bldc_motor_block_commutation.hpp
 *
 * Copyright (C) 2019 Christopher Durand
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef LIBMOTOR_BLDC_MOTOR_BLOCK_COMMUTATION_HPP
#define LIBMOTOR_BLDC_MOTOR_BLOCK_COMMUTATION_HPP

#include <cstdint>

#include "motor_base.hpp"

namespace librobots2::motor
{

/// Control of a BLDC motor with block commutation
template<typename MotorBridge>
class BldcMotorBlockCommutation final : public MotorBase<MotorBridge>
{
public:
	BldcMotorBlockCommutation(uint_fast8_t commutationOffset);

	/// Disables the motor on destruction
	~BldcMotorBlockCommutation();

	/// Set pwm value in range of -32767...32767
	void setSetpoint(int16_t pwm) override;

	void disable() override;

	/// do commutation, can be called from an interrupt context
	void update() override;

	// These cannot be defaulted because the class has volatile member variables
	BldcMotorBlockCommutation(const BldcMotorBlockCommutation&);
	BldcMotorBlockCommutation& operator=(const BldcMotorBlockCommutation&);
private:
	void disableMotor();

	enum class Mode
	{
		Pwm,
		Brake,
		Disabled
	};

	using MotorBase<MotorBridge>::ScaleFactor;

	volatile uint_fast8_t commutationOffset_;
	volatile bool reverse_ = false;
	volatile Mode mode_ = Mode::Disabled;
};

}

#include "bldc_motor_block_commutation_impl.hpp"

#endif // LIBMOTOR_BLDC_MOTOR_BLOCK_COMMUTATION_HPP
