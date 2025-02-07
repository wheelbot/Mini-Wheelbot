/* main.hpp
*
* Copyright (C) 2024 Henrik Hose
* Copyright (C) 2021 Christopher Durand
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

#include <modm/platform.hpp>
#include <modm/processing/timer.hpp>
#include <modm/processing/fiber.hpp>
#include <modm/debug/logger.hpp>

#include "data.hpp"

#include <micro-motor/hardware.hpp>
#include "tasks/can_thread.hpp"
#include "tasks/debug_thread.hpp"
#include "tasks/control_thread.hpp"

#include "tasks/foc_interrupt.hpp"
#include "tasks/encoder_interrupt.hpp"

using namespace std::chrono_literals;

DebugThread debug_thread(main_configuration);
ControlThread control_thread(main_configuration);
CanThread can_thread(main_configuration, control_thread.safety_timeout);

extern modm::Fiber<16000> fiber_can_thread_run;

#undef MODM_LOG_LEVEL
#define MODM_LOG_LEVEL modm::log::INFO

modm_faststack modm::Fiber<16000> fiber_can_thread_run([](){
	can_thread.initialize();
	while(1){
		can_thread.run();
		modm::fiber::yield();
	}
	}, modm::fiber::Start::Later);


modm_faststack modm::Fiber<16000> fiber_debug_thread_run([](){
	while(1){
		debug_thread.run();
		MODM_LOG_DEBUG << "fiber_debug_thread_run.stack_usage() = " << fiber_debug_thread_run.stack_usage() << modm::endl;
		modm::fiber::sleep(main_configuration.dbgtime);
	}
}, modm::fiber::Start::Later);

modm_faststack modm::Fiber<16000> fiber_control_thread_run([](){
	while(1){
		control_thread.run();
		modm::fiber::yield();
	}
}, modm::fiber::Start::Later);

modm_faststack modm::Fiber<16000> fiber_control_thread_calibration([](){
	control_thread.run_cogging_calibration();
}, modm::fiber::Start::Later);

int main()
{
	Board::initializeMcu();
	Board::initializeAllPeripherals();
	Board::Ui::initializeLeds();
	Board::Ui::initializeDebugUart();

	MODM_LOG_INFO << "\nWelcome to the Mini Wheelbot Motor Controller\n" << modm::endl;

	control_thread.initialize_gate_driver();
	if (check_auto_reload()){
		load_from_flash();
	}

	fiber_can_thread_run.watermark_stack();
	fiber_debug_thread_run.watermark_stack();
	fiber_control_thread_run.watermark_stack();
	fiber_control_thread_calibration.watermark_stack();

	fiber_can_thread_run.start();
	fiber_control_thread_run.start();
	fiber_debug_thread_run.start();

	modm::fiber::Scheduler::run();
	while(1);

	return 0;

}
