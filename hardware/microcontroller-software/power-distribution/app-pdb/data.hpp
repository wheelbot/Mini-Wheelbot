/* data.hpp
*
* Copyright (C) 2025 Henrik Hose
* Copyright (C) 2025 Jan Weisgerber
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

#ifndef DATA_HPP
#define DATA_HPP

#include "modm/processing/fiber.hpp"
#include <atomic>

extern "C" const uint32_t __flash_reserved_start[];
constexpr auto max_flash_pages{256};

struct Configuration{

	// debug output mode
	enum class DebugMode{
		Disable,
		Default,
		ScopeIdq,
		ScopePosIq,
	};
	DebugMode dbgmode{DebugMode::Default};
	std::chrono::milliseconds dbgtime{1000};


	// special action states
	enum class SpecialActions : uint8_t {
		None						= 0,
		disableMotors				= 1,
		enableMotors				= 2,
		processBitmask				= 3,
	};
	SpecialActions special_action{SpecialActions::None};
	uint8_t special_action_progress{0};
	uint32_t special_action_value{0};

	// buzzer states
	enum class BuzzerActions : uint8_t {
		None						= 0,
		disableBuzzer				= 1,
		enableBuzzer				= 2,
		finishedBuzzing				= 3,
		buzzingEnableMotor			= 4,
		buzzingShortCircuitBreak	= 5,
	};
	BuzzerActions buzzer_action{BuzzerActions::None};

	struct TempData{
		bool chargingStatus{false};
		bool driveEnabled{false};
	};
	TempData temp_data;

	struct PersistentData{
		std::optional<uint8_t> board_id{std::nullopt}; // [1...7]
	};
	PersistentData persistent_data;
};

static Configuration main_configuration{};


#endif // DATA_HPP
