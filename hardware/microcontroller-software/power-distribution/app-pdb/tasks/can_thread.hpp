/* can_thread.hpp
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

#ifndef CAN_THREAD_HPP
#define CAN_THREAD_HPP

#include <modm/processing.hpp>
#include "../board.hpp"
#include "../pdb_can.hpp"
#include "../data.hpp"

using namespace Board;


class CanThread
{
public:
	CanThread(Configuration &config, modm::PreciseTimeout &safety_timeout) : config(config), safety_timeout(safety_timeout) {};

	void
	run()
	{
			using namespace pdbCan;

			std::visit(overloaded {
				[](std::monostate) {}, // No message
				[this](const pdbCan::Sync&) { this->syncReceived(); },
				[&](const RequestToPdb& command) { processRequest(command); },
			},
			getCanMessage<Board::CanBus::Can>()
			);

			// publishVoltage(); // Turn on to publish voltage to can
	}

	void initialize()
	{
		pdbCan::setupCanFilters<Board::CanBus::Can>();
		// MODM_LOG_INFO.printf("ToDo");
	}

void syncReceived()
	{
		Board::Ui::LedGreen::toggle();
		publishVoltageCurrent();
	}

void processRequest(const pdbCan::RequestToPdb& c){
		switch (c.command)
		{
			case pdbCan::RequestToPdb::Commands::disableMotors:
				config.special_action=Configuration::SpecialActions::disableMotors;
				break;

			case pdbCan::RequestToPdb::Commands::enableMotors:
				config.special_action=Configuration::SpecialActions::enableMotors;
				break;

			case pdbCan::RequestToPdb::Commands::processBitmask:
				config.special_action=Configuration::SpecialActions::processBitmask;
				config.special_action_value=c.value;
				break;
		}
	}

	Configuration &config;
	modm::PreciseTimeout &safety_timeout;


void publishVoltageCurrent()
	{
		pdbCan::DataFromPDB data{};
		data.voltage = Adc1::getValue() * DrivePower::v_supply_div;
		data.current = (Adc2::getValue() - 0x7ff) * DrivePower::i_drive_div;
		pdbCan::sendResponse<Board::CanBus::Can>(data);
	}
};

#endif // CAN_THREAD_HPP
