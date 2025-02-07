/* motor_base.hpp
 *
 * Copyright (C) 2019 Christopher Durand
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef LIBMOTOR_MOTOR_BASE_HPP
#define LIBMOTOR_MOTOR_BASE_HPP

#include <cstdint>
#include <limits>

#include "motor_interface.hpp"

namespace librobots2::motor
{

template<typename MotorBridge>
class MotorBase : public MotorInterface
{
public:
	virtual ~MotorBase() = default;

	static constexpr bool isPowerOf2(uint32_t n) { return ((n == 0) || ((n & (n-1)) == 0)); }

	static_assert((MotorBridge::MaxPwm <= 32767) && (MotorBridge::MaxPwm >= 3));
	static_assert(isPowerOf2(MotorBridge::MaxPwm + 1), "(MaxPwm + 1) must be a power of 2!");

	static constexpr int ScaleFactor =
		(std::numeric_limits<int16_t>::max() + 1) / (MotorBridge::MaxPwm + 1);

	static_assert(isPowerOf2(ScaleFactor));
};

}

#endif // LIBMOTOR_MOTOR_BASE_HPP
