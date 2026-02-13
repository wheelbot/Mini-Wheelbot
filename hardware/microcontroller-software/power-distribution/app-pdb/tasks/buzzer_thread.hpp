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

#ifndef BUZZER_THREAD_HPP
#define BUZZER_THREAD_HPP

#include <modm/processing.hpp>
#include "../data.hpp"
#include "../board.hpp"


using namespace Board;

extern modm::Fiber<2048> fiber_buzzer_thread_calibration;
class BuzzerThread
{
public:
	BuzzerThread(Configuration &config) : config(config) {}

	void run()
	{
		if(config.buzzer_action == Configuration::BuzzerActions::disableBuzzer){
            disable_buzzer();
            config.buzzer_action = Configuration::BuzzerActions::None;
        }
		if(config.buzzer_action == Configuration::BuzzerActions::enableBuzzer){
            enable_buzzer();
            config.buzzer_action = Configuration::BuzzerActions::None;
        }
		if(config.buzzer_action == Configuration::BuzzerActions::buzzingEnableMotor){
            buzzing_enable_motor();
        }
		if(config.buzzer_action == Configuration::BuzzerActions::buzzingShortCircuitBreak){
            buzzing_short_circuit_break();
            config.buzzer_action = Configuration::BuzzerActions::None;
        }
	}

	void
	disable_buzzer(){
		Ui::buzzerOff();
	}

	void
	enable_buzzer(){
		Ui::buzzerOn();
	}

	void
	buzzing_enable_motor(){
		for (int i = 0; i<10; i++){
			Ui::buzzerOn();
			modm::this_fiber::sleep_for(50ms);
			Ui::buzzerOff();
			modm::this_fiber::sleep_for(50ms);
		}
		// after the arming signal is played, activate the drive power
		// Timer16::setCompareValue<GpioA6::Ch1>(Timer16::getOverflow());
    	config.buzzer_action = Configuration::BuzzerActions::finishedBuzzing;
	}

	void
	buzzing_short_circuit_break(){
		for (int i = 0; i<5; i++){
			Ui::buzzerOn();
			modm::this_fiber::sleep_for(200ms);
			Ui::buzzerOff();
			modm::this_fiber::sleep_for(200ms);
		}
	}

private:
	Configuration &config;
};

#endif // BUZZER_THREAD_HPP
