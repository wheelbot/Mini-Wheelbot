/*
 * Copyright (C) 2019 Raphael Lehmann <raphael@rleh.de>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef MOTOR_CAN_MOTOR_BOARD_HPP
#define MOTOR_CAN_MOTOR_BOARD_HPP

namespace motorCan
{

/**
 * Interface to be provided by the motor board
 */
class MotorBoard
{
public:
	enum class Motor{
		M1,
		M2,
	};

	static
	uint16_t
	getRawEncoderValue(Motor motor);

	static
	int16_t
	getCurrent(Motor motor);

	static
	void
	disable(Motor motor);

	static
	void
	setPwm(Motor motor, int16_t pwm);

	static
	void
	setCurrentLimit(Motor motor, int16_t currentLimit);
};

} // namespace motorCan

#endif // MOTOR_CAN_MOTOR_BOARD_HPP
