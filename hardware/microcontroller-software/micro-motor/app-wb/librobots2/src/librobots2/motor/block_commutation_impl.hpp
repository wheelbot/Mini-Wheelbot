/* block_commutation_impl.hpp
 *
 * Copyright (C) 2019 Christopher Durand
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef LIBMOTOR_BLOCK_COMMUTATION_HPP
#error "Do not use this file directly, include 'block_commutation.hpp' instead!"
#endif

#include "block_commutation_private.hpp"
#include "hall_permutations.hpp"

namespace librobots2::motor
{

template<typename HallPort>
BridgeConfig
BlockCommutation<HallPort>::doCommutation(uint_fast8_t commutationOffset, bool reverse)
{
	constexpr auto& HallLut = block_commutation::SequenceLut;
	constexpr auto& CommutationLut = block_commutation::CommutationLut;

	using Hall = HallPermutations<HallPort>;

	static_assert(HallLut.size() == 8);
	static_assert(CommutationLut.size() == 8);

	uint_fast8_t hallValue = Hall::read(commutationOffset);
	if(reverse) {
		hallValue = ~hallValue;
	}
	hallValue &= 0b111;

	return CommutationLut[HallLut[hallValue]];
}

template<typename HallPort>
BridgeConfig
BlockCommutation<HallPort>::disable()
{
	constexpr auto& Lut = block_commutation::CommutationLut;
	static_assert(Lut.size() == 8);
	return Lut[6]; // index 6: disabled (HiZ, HiZ, HiZ)
}

template<typename HallPort>
BridgeConfig
BlockCommutation<HallPort>::motorBrake()
{
	constexpr auto& Lut = block_commutation::CommutationLut;
	static_assert(Lut.size() == 8);
	return Lut[7]; // index 7: brake (low, low, low)
}

}
