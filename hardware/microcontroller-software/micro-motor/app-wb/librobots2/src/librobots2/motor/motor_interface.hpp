/* motor_interface.hpp
 *
 * Copyright (C) 2019 Christopher Durand
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef LIBMOTOR_MOTOR_INTERFACE_HPP
#define LIBMOTOR_MOTOR_INTERFACE_HPP

namespace librobots2::motor
{

class MotorInterface
{
public:
	virtual ~MotorInterface() = default;

	/// Set pwm value in range of -32767...32767
	virtual void setSetpoint(int16_t pwm) = 0;

	virtual void disable() = 0;

	virtual void update() = 0;
};

}

#endif // LIBMOTOR_MOTOR_INTERFACE_HPP
