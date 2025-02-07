/* motor_bridge.hpp
 *
 * Copyright (C) 2019 Christopher Durand
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef LIBMOTOR_MOTOR_BRIDGE_HPP
#define LIBMOTOR_MOTOR_BRIDGE_HPP

#include <cstdint>
#include <array>

namespace librobots2::motor
{

enum class Phase : uint_fast8_t
{
	PhaseU = 0,
	PhaseV = 1,
	PhaseW = 2
};

enum class
PhaseConfig : uint_fast8_t
{
	HiZ,
	Pwm,
	Low,
	High
};

struct BridgeConfig
{
	PhaseConfig get(Phase) const;

	std::array<PhaseConfig, 3> config;
};

#ifdef DOXYGEN
/// Three-phase motor bridge interface, for documentation purposes only
class ThreePhaseBridge
{
public:
	// Maximum pwm compare value, must be (2**N - 1) with N integral, N=2..15
	static constexpr uint16_t MaxPwm = 1023;

	/// InitiAalize, must be called before any other members are accessed
	static void initialize();

	/// Configure all phases
	static void configure(const BridgeConfig& config);

	/// Configure all phases with the same mode
	static void configure(PhaseConfig config);

	/// Configure a single phase
	static void configure(Phase phase, PhaseConfig config);

	/// Set pwm compare value for all phases
	static void setCompareValue(uint16_t compareValue);

	/// Set pwm compare value for a single phase
	static void setCompareValue(Phase phase, uint16_t compareValue);

	/// Apply set compare values
	static void applyCompareValues();
};
#endif

}

#endif // LIBMOTOR_MOTOR_BRIDGE_HPP
