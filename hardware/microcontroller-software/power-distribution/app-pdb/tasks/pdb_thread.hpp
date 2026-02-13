/* control_thread.hpp
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

#ifndef PDB_THREAD_HPP
#define PDB_THREAD_HPP

// #include <modm/processing.hpp>
#include "modm/processing.hpp"
#include "../data.hpp"
#include "../board.hpp"


using namespace Board;

class Charger{
public:
	void update(Configuration &config){
		if (Charging::ChargingPin::read())
		{
            config.temp_data.chargingStatus = true;
			DrivePower::disableDrive();
			LogicPower::EnableMotor1::reset();
			LogicPower::EnableMotor2::reset();
		}
	}
private:
	bool last_state{false};
};

extern modm::Fiber<2048> fiber_pdb_thread_calibration;
class PDBThread
{
public:
	PDBThread(Configuration &config) : config(config), safety_timeout(std::chrono::milliseconds(200)) {}

	void run()
	{
		charger.update(config);
		// Ui::LedRed::toggle();
		// float i_drive  = (Adc2::getValue() - 0x7ff) * DrivePower::i_drive_div;
		// MODM_LOG_INFO << "Measured drive current: " << i_drive << " A" << modm::endl;
	}



	void initialize()
	{
		Charging::initializeCharging();
		// if (!Charging::ChargingPin::read()){
		// 	DrivePower::enableDrive();
		// 	LogicPower::EnableMotor1::set();
		// 	LogicPower::EnableMotor2::set();
		// }
	}

	modm::PreciseTimeout safety_timeout{std::chrono::milliseconds(200)};

private:
	Charger charger;
	Configuration &config;
};

#endif // PDB_THREAD_HPP
