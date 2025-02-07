/*
 * Copyright (C) 2019 Raphael Lehmann <raphael@rleh.de>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef MOTOR_CAN_MOTOR_CONFIGURATION_HPP
#define MOTOR_CAN_MOTOR_CONFIGURATION_HPP

#include <stdint.h>
#include <modm/io/iostream.hpp>

#include <array>

namespace motorCan
{

enum class BoardConfig : uint8_t
{
	DoubleBLDC,
	DoubleDC,
	SingleBLDCSingleDC,
};

class Configuration
{
public:
	// A board is a microcontroller two motors (or less)
	static constexpr uint8_t
	BoardCount =  5;

	// All motors are enumerated from 0 to MotorCount - 1.
	static constexpr uint8_t
	MotorCount = BoardCount * 2;

public:
	// ID of the sync packet
	static constexpr uint16_t
	sync_id = 0x0000;

	// Length of the sync packet
	static constexpr uint8_t
	sync_length = 0;

	// ID of the first motor
	static constexpr uint16_t
	base_id = 0x10;

	// ID of the first motor reply packets
	static constexpr uint16_t
	base_id_reply = 0x80;
};

} // namespace motorCan


/*
inline const char*
enumToString(motorCan::BoardConfig b)
{
	switch (b) {
		case motorCan::BoardConfig::DoubleBLDC: return "DoubleBLDC";
		case motorCan::BoardConfig::DoubleDC: return "DoubleDC";
		case motorCan::BoardConfig::SingleBLDCSingleDC: return "SingleBLDCSingleDC";
		default: return "__UNKNOWN__";
	}
}

namespace motorCan
{

inline ::modm::IOStream&
operator << (::modm::IOStream& s, const motorCan::BoardConfig b)
{
	s << enumToString(b);
	return s;
}

} // namespace motorCan
*/

#endif // MOTOR_CAN_MOTOR_CONFIGURATION_HPP
