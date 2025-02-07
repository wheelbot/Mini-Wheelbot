/* debug_thread.hpp
*
* Copyright (C) 2024 Henrik Hose
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
#include <modm/debug/logger.hpp>
#include "../data.hpp"

class DebugThread
{
public:
	DebugThread(Configuration &config) : config(config) {}

	void run()
	{

		if ( config.dbgmode ==Configuration::DebugMode::ScopeIdq ){
			MODM_LOG_DEBUG << "dqc:"
				<< 0*config.foc.current_D_out.load(std::memory_order_relaxed) << ","
				<< config.foc.current_Q_out.load(std::memory_order_relaxed) << ","
				<< config.foc.commanded_torque.load(std::memory_order_relaxed) << "," << modm::endl;
				MODM_LOG_DEBUG.flush();
		}
		else if (config.dbgmode ==Configuration::DebugMode::ScopePosIq){
			MODM_LOG_DEBUG << "dqc:"
				<< config.foc.posctrl.setpoint.load(std::memory_order_relaxed) << ","
				<< config.encoder.angle_degrees_aligned_with_electrical_frame.load(std::memory_order_relaxed) << ","
				<< config.foc.commanded_torque.load(std::memory_order_relaxed) <<","<< modm::endl;
				MODM_LOG_DEBUG.flush();
		}
		else if (config.dbgmode ==Configuration::DebugMode::Default){
			MODM_LOG_DEBUG << modm::endl;
			MODM_LOG_DEBUG << "Debug Thread:" << modm::endl;
			// MODM_LOG_DEBUG << "      encoder raw_value: " << ( config.encoder.inbuf & 0x3fff );
			MODM_LOG_DEBUG << "   encoder    raw: " << config.encoder.raw_value.load(std::memory_order_relaxed) ;
			MODM_LOG_DEBUG << "    position: " << config.encoder.position.load(std::memory_order_relaxed) ;
			// MODM_LOG_DEBUG << "    diff: " << config.encoder.diff.load(std::memory_order_relaxed) ;
			MODM_LOG_DEBUG.printf("    aligned: %.3f [deg]", config.encoder.angle_degrees_aligned_with_electrical_frame.load(std::memory_order_relaxed));
			MODM_LOG_DEBUG << "    corrupted: " << config.encoder.corrupted_data_counter.load(std::memory_order_relaxed) ;
			MODM_LOG_DEBUG << modm::endl;

			MODM_LOG_DEBUG.printf("   current     U: %.3f [A]", config.foc.current_U.load(std::memory_order_relaxed)     );
			MODM_LOG_DEBUG.printf("    V: %.3f [A]", config.foc.current_V.load(std::memory_order_relaxed)     );
			MODM_LOG_DEBUG.printf("    W: %.3f [A]\n", config.foc.current_W.load(std::memory_order_relaxed)     );
			MODM_LOG_DEBUG.printf("   voltage Vin: %.3f [V]\n", config.foc.voltage_Vin.load(std::memory_order_relaxed) );
			MODM_LOG_DEBUG.printf("   foc: V_D: %.3f [V]",config.foc.voltage_integral_D.load(std::memory_order_relaxed));
			MODM_LOG_DEBUG.printf("  V_Q: %.3f [V]",config.foc.voltage_integral_Q.load(std::memory_order_relaxed));
			MODM_LOG_DEBUG.printf("  I_D: %.3f [A]",config.foc.current_D_out.load(std::memory_order_relaxed));
			MODM_LOG_DEBUG.printf("  I_Q: %.3f [A]",config.foc.current_Q_out.load(std::memory_order_relaxed));
			MODM_LOG_DEBUG.printf("  I_cmd: %.3f [A]\n",config.foc.commanded_torque.load(std::memory_order_relaxed));
			MODM_LOG_DEBUG << modm::endl;
			MODM_LOG_DEBUG.flush();
		}
		else{

		}

	};

private:
	Configuration &config;
	modm::ShortTimeout timeout;
};


#endif // DEBUG_THREAD_HPP
