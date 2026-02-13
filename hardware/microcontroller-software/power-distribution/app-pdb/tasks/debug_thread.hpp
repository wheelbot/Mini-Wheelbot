/* debug_thread.hpp
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

#ifndef DEBUG_THREAD_HPP
#define DEBUG_THREAD_HPP

#include <modm/processing.hpp>
#include "../data.hpp"
#include "../board.hpp"


using namespace Board;

extern modm::Fiber<2048> fiber_debug_thread_calibration;
class DebugThread
{
public:
	DebugThread(Configuration &config) : config(config) {}

	void run()
	{
		Ui::LedRed::toggle();

		int adcValue = Adc1::getValue();
		int adc2Value = Adc2::getValue();
		float v_supply = Adc1::getValue() * DrivePower::v_supply_div;
		float i_drive  = (Adc2::getValue() - 0x7ff) * DrivePower::i_drive_div;
		MODM_LOG_INFO << "adcValue=" << adcValue;
		// float voltage = adcValue * 2.9f / 0xfff * 11.f/0.968;
		MODM_LOG_INFO << "in voltage=";
		MODM_LOG_INFO.printf("%.3f", v_supply);
		MODM_LOG_INFO << " adc2Value=" << (adc2Value-0x7ff);
		MODM_LOG_INFO << " in current=";
		MODM_LOG_INFO.printf("%.3f", i_drive);
		// braking_pid_value = braking_pid.getValue();
		// MODM_LOG_INFO << " brakingpid=";
		// MODM_LOG_INFO.printf("%.3f", braking_pid_value);
		MODM_LOG_INFO << modm::endl;
	}

private:
	Configuration &config;
};

#endif // DEBUG_THREAD_HPP
