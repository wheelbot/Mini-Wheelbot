/* utils.hpp
*
* Copyright (C) 2024 Henrik Hose
* Copyright (C) 2021 Christopher Durand
* Copyright (C) 2021 Sebastian Birke
* Copyright (C) 2018 Raphael Lehmann <raphael@rleh.de>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MY_UTILS_HPP
#define MY_UTILS_HPP

#include <modm/platform.hpp>
#include <modm/debug/logger.hpp>
#include <modm/io/iostream.hpp>

uint32_t readHardwareId()
{
	return *((volatile uint32_t *) UID_BASE);
    // return 42;
}

uint8_t readBoardId()
{
	static constexpr std::array boards = {
		// hardware id, board id
		std::pair{0x003f004eu, 1u},
		std::pair{0x00180032u, 2u},
		std::pair{0x001e002bu, 3u},
		std::pair{1966129u, 3u}
	};

	const auto hardwareId = readHardwareId();
	MODM_LOG_INFO << "Board ID is " << hardwareId << modm::endl;
	auto it = std::find_if(std::begin(boards), std::end(boards), [hardwareId](auto board) {
		return board.first == hardwareId;
	});
	if (it == std::end(boards)) {
    MODM_LOG_INFO << "Board not found" << modm::endl;
		// while(1) asm volatile("nop");
		return 42;
	}

	return it->second;
}

constexpr uint16_t
checkMsbEvenParity(const uint16_t num)
{
    uint16_t par = 0x7fff & num;
    par ^= par >> 8;
    par ^= par >> 4;
    par ^= par >> 2;
    par ^= par >> 1;
    return (( num >> 15 ) & 1) == ( par & 1 );
}

// Modulo (as opposed to remainder), per https://stackoverflow.com/a/19288271
inline int mod(const int dividend, const int divisor){
    int r = dividend % divisor;
    if (r < 0) r += divisor;
    return r;
}

#endif // MY_UTILS_HPP
