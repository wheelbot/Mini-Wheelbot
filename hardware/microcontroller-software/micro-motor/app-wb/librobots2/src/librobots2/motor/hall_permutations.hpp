/* hall_permutations.hpp
 *
 * Copyright (C) 2019 Christopher Durand
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef LIBMOTOR_HALL_PERMUTATIONS_HPP
#define LIBMOTOR_HALL_PERMUTATIONS_HPP

#include <tuple>
#include <cstdint>

namespace librobots2::motor
{

template<typename Port>
class HallPermutations;

template<template<typename...> typename Port,
	typename PinA,
	typename PinB,
	typename PinC
>
class HallPermutations<Port<PinA, PinB, PinC>>
{
	// Possible permutations of hall sensors
	using P = std::tuple<
		Port<PinA, PinB, PinC>,
		Port<PinA, PinC, PinB>,
		Port<PinB, PinA, PinC>,
		Port<PinB, PinC, PinA>,
		Port<PinC, PinA, PinB>,
		Port<PinC, PinB, PinA>
	>;

public:
	static auto
	read(uint_fast8_t offset)
	{
		if(offset == 0) {
			return std::tuple_element_t<0, P>::read();
		} else if(offset == 1) {
			return std::tuple_element_t<1, P>::read();
		} else if(offset == 2) {
			return std::tuple_element_t<2, P>::read();
		} else if(offset == 3) {
			return std::tuple_element_t<3, P>::read();
		} else if(offset == 4) {
			return std::tuple_element_t<4, P>::read();
		} else {
			return std::tuple_element_t<5, P>::read();
		}
	}
};

}

#endif // LIBMOTOR_HALL_PERMUTATIONS_HPP
