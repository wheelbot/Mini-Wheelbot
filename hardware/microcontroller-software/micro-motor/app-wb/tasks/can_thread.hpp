/* can_thread.hpp
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

#ifndef CAN_THREAD_HPP
#define CAN_THREAD_HPP

#include <modm/processing.hpp>
#include <micro-motor/hardware.hpp>
#include "../motor_can.hpp"
#include "../data.hpp"
#include <librobots2/motion/encoder/atomic_encoder.hpp>
#include <micro-motor/wheelbot/wheelbot_hardware.hpp>

class CanThread
{
public:
	CanThread(Configuration &config, modm::PreciseTimeout &safety_timeout) : config(config), safety_timeout(safety_timeout) {};

	void
	run()
	{
			using namespace motorCan;

			std::visit(overloaded {
				[](std::monostate) {}, // No message
				[&](const Sync&) { syncReceived(config.persistent_data.board_id); },
				[&](const TorqueToMotor&  command)  { processCommand(command); },
				[&](const DiagnosticRequestToMotor& command) { processConfig(command); },
				[&](const PositionToMotor& command) { processPosition(command); },
			},
			getCanMessage<Board::CanBus::Can>(config.persistent_data.board_id));
	}

	void initialize()
	{
		if ( config.persistent_data.board_id.has_value())
			MODM_LOG_INFO.printf("Board ID: %d \n\n", config.persistent_data.board_id.value());
		else{
			MODM_LOG_INFO.printf("Board ID: value not set! Hardware ID is 0x%08lx\n\n", readHardwareId());
		}
		motorCan::setupCanFilters<Board::CanBus::Can>(config.persistent_data.board_id);
	}

private:
	void syncReceived(std::optional<uint8_t> boardId)
	{
		Board::Ui::LedGreen::toggle();
		if (boardId.has_value()){
			motorCan::DataFromMotor data{};
			data.angle = config.encoder.angle_degrees_aligned_with_electrical_frame;
			while(data.angle >  180.f) data.angle -= 360.f;
			while(data.angle < -180.f) data.angle += 360.f;
			motorCan::sendResponse<Board::CanBus::Can>(data, boardId);
		}
	}

	void processCommand(const motorCan::TorqueToMotor& command)
	{
		config.foc.commanded_torque = command.u;
		config.foc.foc_state = Configuration::FocState::CurrentControl;
		safety_timeout.restart();
	}

	void processPosition(const motorCan::PositionToMotor& command)
	{
		config.foc.posctrl.setpoint = command.u;
		config.foc.control_state = Configuration::ControlState::PositionControl;
		safety_timeout.restart();
	}

	void processConfig(const motorCan::DiagnosticRequestToMotor& c){
		switch (c.command)
		{
			case motorCan::DiagnosticRequestToMotor::Commands::SetBoardId:
				if (c.value == readHardwareId() && c.second_value < 8){
					MODM_LOG_INFO << "New board ID received: " << c.second_value << modm::endl;
					config.persistent_data.board_id = static_cast<uint8_t>(c.second_value);
					motorCan::setupCanFilters<Board::CanBus::Can>(config.persistent_data.board_id);
				}
				break;

			case motorCan::DiagnosticRequestToMotor::Commands::AbortCalibration:
				MODM_LOG_INFO << "Received Abort!!!"<< modm::endl;
				config.special_action=Configuration::SpecialActions::Abort;
				break;

			case motorCan::DiagnosticRequestToMotor::Commands::CalibrateEncoder:
				config.special_action=Configuration::SpecialActions::EncoderCalibrationStart;
				break;

			case motorCan::DiagnosticRequestToMotor::Commands::CalibrateCogging:
				config.special_action=Configuration::SpecialActions::CoggingCalibrationStart;
				break;

			case motorCan::DiagnosticRequestToMotor::Commands::SetCoggingCalibrationToZero:
				MODM_LOG_INFO << "Setting cogging torques to zero!" << modm::endl;
				for (auto& d : config.persistent_data.feedforward_cogging_torques) d = 0;
				// save_to_flash();
				break;

			case motorCan::DiagnosticRequestToMotor::Commands::EnableCoggingCompensation:
				MODM_LOG_INFO << "Switching on compensation!!!"<< modm::endl;
				config.persistent_data.use_cogging_compensation = true;
				// save_to_flash();
				break;

			case motorCan::DiagnosticRequestToMotor::Commands::DisableCoggingCompensation:
				MODM_LOG_INFO << "Switching off compensation!!!"<< modm::endl;
				config.persistent_data.use_cogging_compensation = false;
				// save_to_flash();
				break;

			case motorCan::DiagnosticRequestToMotor::Commands::SaveConfigToFlash:
				save_to_flash();
				break;

			case motorCan::DiagnosticRequestToMotor::Commands::LoadConfigFromFlash:
				load_from_flash();
				break;

			case motorCan::DiagnosticRequestToMotor::Commands::SetAutoReload:
				MODM_LOG_INFO << "Enabled auto_reload"<< modm::endl;
				config.persistent_data.auto_reload=1;
				// save_to_flash();
				break;

			case motorCan::DiagnosticRequestToMotor::Commands::ResetAutoReload:
			MODM_LOG_INFO << "Disabled auto_reload"<< modm::endl;
				config.persistent_data.auto_reload=0;
				// save_to_flash();
				break;

			case motorCan::DiagnosticRequestToMotor::Commands::DebugDisableRegularPrinting:
				config.dbgmode = Configuration::DebugMode::Disable;
				// config.dbgtime = std::chrono::milliseconds(1000);
				break;

			case motorCan::DiagnosticRequestToMotor::Commands::DebugModeDefault:
				config.dbgmode = Configuration::DebugMode::Default;
				config.dbgtime = std::chrono::milliseconds(1000);
				break;

			case motorCan::DiagnosticRequestToMotor::Commands::DebugModeScopeIdq:
				config.dbgmode = Configuration::DebugMode::ScopeIdq;
				config.dbgtime = std::chrono::milliseconds(1);
				break;

			case motorCan::DiagnosticRequestToMotor::Commands::DebugModeScopePosIq:
				config.dbgmode = Configuration::DebugMode::ScopePosIq;
				config.dbgtime = std::chrono::milliseconds(1);
				break;

			case motorCan::DiagnosticRequestToMotor::Commands::ReturnCoggingData:
				for (size_t idx{0}; idx < config.persistent_data.feedforward_cogging_torques.size(); idx++){
					motorCan::DiagnosticResponseFromMotor response{
						motorCan::DiagnosticResponseFromMotor::ControlWord::Data,
						std::make_pair(idx,config.persistent_data.feedforward_cogging_torques[idx])};
					motorCan::sendResponse<Board::CanBus::Can>(response, config.persistent_data.board_id);
					modm::fiber::sleep(std::chrono::microseconds(100));
					// modm::fiber::yield();
				}
				break;

			case motorCan::DiagnosticRequestToMotor::Commands::ReturnEncoderErrorCounter:
				{
					motorCan::DiagnosticResponseFromMotor response{
						motorCan::DiagnosticResponseFromMotor::ControlWord::Data, config.encoder.corrupted_data_counter.load(std::memory_order_relaxed)
					};
					motorCan::sendResponse<Board::CanBus::Can>(response, config.persistent_data.board_id);
				}
				break;


			case motorCan::DiagnosticRequestToMotor::Commands::ReturnSpecialActionStatus:
				{
					motorCan::DiagnosticResponseFromMotor response{
						motorCan::DiagnosticResponseFromMotor::ControlWord::Data,
						uint16_t{ (static_cast<uint16_t>(config.special_action)<<8) |  static_cast<uint8_t>(config.special_action_progress)}};
					motorCan::sendResponse<Board::CanBus::Can>(response, config.persistent_data.board_id);
				}
				break;

			case motorCan::DiagnosticRequestToMotor::Commands::ReturnHardwareUuid:
				{
					motorCan::DiagnosticResponseFromMotor response{
						motorCan::DiagnosticResponseFromMotor::ControlWord::Data, readHardwareId()};
					motorCan::sendResponse<Board::CanBus::Can>(response, config.persistent_data.board_id);
				}
				break;
		}
	}

	Configuration &config;
	modm::PreciseTimeout &safety_timeout;
};

#endif // CAN_THREAD_HPP
