/* dc_motor.hpp
 *
 * Copyright (C) 2019 Christopher Durand
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef LIBMOTOR_DC_MOTOR_HPP
#define LIBMOTOR_DC_MOTOR_HPP

#include <cstdint>

#include "motor_base.hpp"

namespace librobots2::motor
{

/// Control of a DC motor connected to two phases of a three-phase bridge
template<typename MotorBridge>
class DcMotor final : public MotorBase<MotorBridge>
{
public:
	DcMotor();

	/// Disables the motor on destruction
	~DcMotor();

	DcMotor(const DcMotor&) = default;
	DcMotor& operator=(const DcMotor&) = default;

	DcMotor(DcMotor&&) = default;
	DcMotor& operator=(DcMotor&&) = default;

	/// Set pwm value in range of -32767...32767
	void setSetpoint(int16_t pwm) override;

	void disable() override;

	/// no-op for dc motor
	void update() override {}

private:
	void disableMotor();

	using MotorBase<MotorBridge>::ScaleFactor;
};

}

#include "dc_motor_impl.hpp"

#endif // LIBMOTOR_MOTOR_BRIDGE_HPP
