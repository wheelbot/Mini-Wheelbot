/* motor_disabled.hpp
 *
 * Copyright (C) 2019 Christopher Durand
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef LIBMOTOR_MOTOR_DISABLED_HPP
#define LIBMOTOR_MOTOR_DISABLED_HPP

#include <cstdint>

#include "motor_base.hpp"
#include "motor_bridge.hpp"

namespace librobots2::motor
{

/// Dummy motor which disables all outputs
template<typename MotorBridge>
class MotorDisabled final : public MotorInterface
{
public:
	MotorDisabled() { disableMotors(); }

	~MotorDisabled() { disableMotors(); }

	inline void setSetpoint([[maybe_unused]] int16_t pwm) override {};

	inline void disable() override { disableMotors(); };

	inline void update() override {}

private:
	void disableMotors()
	{
		MotorBridge::configure(PhaseConfig::HiZ);
		MotorBridge::setCompareValue(0);
	}
};

}

#endif // LIBMOTOR_MOTOR_DISABLED_HPP
