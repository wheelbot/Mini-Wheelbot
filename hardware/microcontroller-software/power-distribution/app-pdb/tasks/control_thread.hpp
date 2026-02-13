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

#ifndef CONTROL_THREAD_HPP
#define CONTROL_THREAD_HPP

#include <modm/processing.hpp>
#include "../data.hpp"
#include "../board.hpp"

using namespace Board;

extern modm::Fiber<2048> fiber_control_thread_calibration;
class ControlThread
{
public:
	ControlThread(Configuration &config) : config(config), safety_timeout(std::chrono::milliseconds(200)) {}

	void
	run()
	{
        if(config.special_action == Configuration::SpecialActions::disableMotors){
            config.special_action = Configuration::SpecialActions::None;
            config.buzzer_action = Configuration::BuzzerActions::None;
            disable_motors();
        }
        if(config.special_action == Configuration::SpecialActions::enableMotors){
            if(config.buzzer_action == Configuration::BuzzerActions::None){
                config.buzzer_action = Configuration::BuzzerActions::buzzingEnableMotor;
            }
            if(config.buzzer_action == Configuration::BuzzerActions::finishedBuzzing){
                config.special_action = Configuration::SpecialActions::None;
                config.buzzer_action = Configuration::BuzzerActions::None;
                if (!Charging::ChargingPin::read()) {
                    enable_motors();
                }
            }
        }
        if(config.special_action == Configuration::SpecialActions::processBitmask){
            /*
                The CAN payload looks like -- 03 0a 0b 0c --

                The bitmask is structured as follows:
                    - 03: calls processBitmask
                    - 0a: if set, the drive power is activated
                    - 0b: if set, the logic power is activated for motor 1
                    - 0c: if set, the logic power is activated for motor 2

                Example: -- 03 00 01 01 -- enables the logic power for both motors, but the drive power is disabled.
            */
            config.special_action = Configuration::SpecialActions::None;

            if (config.special_action_value & (1 << 16)) {
                if (!Charging::ChargingPin::read()) {
                    config.special_action = Configuration::SpecialActions::enableMotors;
                }
            } else {
                DrivePower::disableDrive();
            }

            if (config.special_action_value & (1 << 8)) {
                LogicPower::EnableMotor1::set();
            } else {
                LogicPower::EnableMotor1::reset();
            }

            if (config.special_action_value & (1 << 0)) {
                LogicPower::EnableMotor2::set();
            } else {
                LogicPower::EnableMotor2::reset();
            }

            config.special_action_value = 0;

        }
		else if (safety_timeout.isArmed()){

		}

	}

    void
    disable_motors(){
        DrivePower::disableDrive();
        LogicPower::EnableMotor1::reset();
        LogicPower::EnableMotor2::reset();
    }


    void
    enable_motors(){
        DrivePower::enableDrive();
        LogicPower::EnableMotor1::set();
        LogicPower::EnableMotor2::set();
    }


	modm::PreciseTimeout safety_timeout;

private:
	Configuration &config;
};

#endif // CONTROL_THREAD_HPP
