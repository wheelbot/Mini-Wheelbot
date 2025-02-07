/* control_thread.hpp
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

#ifndef CONTROL_THREAD_HPP
#define CONTROL_THREAD_HPP

#include <modm/processing.hpp>
#include <modm/driver/motor/drv832x_spi.hpp>
#include "../data.hpp"
#include "../encoder.hpp"

constexpr
Configuration::BldcMotorParameters mn4006Parameters{
	.inductance = 80.e-6f,
	.resistance = 150.e-3f,
	.electrical_per_mechanical_rev = 12.f};

constexpr
Configuration::CurrentSenseParameters wbPcb{
	.reference_voltage = 3.0f,
	.amp_gain=20,
	.adc_resolution=float(1<<12),
	.shunt_resistance=3.e-3f};

constexpr
Configuration::ControlLoopParameters controlLoopParameters{
	.current_control_loop_rate = 83.e3f/2.f,
	.bandwidth= 5000.f,
};

extern modm::Fiber<16000> fiber_control_thread_calibration;
class ControlThread
{
public:
	ControlThread(Configuration &config) : config(config), safety_timeout(std::chrono::milliseconds(200)) {}

	void
	run()
	{
		if (config.special_action == Configuration::SpecialActions::EncoderCalibrationStart){
			MODM_LOG_INFO << "Entering encoder Offset Calibration" << modm::endl;
			config.foc.foc_state = Configuration::FocState::Disable;
			config.special_action = Configuration::SpecialActions::EncoderCalibrationRunning;
			run_encoder_offset_calibration();
			config.special_action = Configuration::SpecialActions::EncoderCalibrationSuccess;
		}
		else if (config.special_action == Configuration::SpecialActions::CoggingCalibrationStart){
			config.special_action = Configuration::SpecialActions::CoggingCalibrationRunning;
			fiber_control_thread_calibration.start();
		}

		if ((config.encoder.corrupted_data_counter) > 10'000 || ( config.encoder.not_connected_counter > 40'000 )){
			MODM_LOG_WARNING << "Aborting due to high corrupted encoder counter: "
				<< config.encoder.corrupted_data_counter.load(std::memory_order_relaxed) << " or high not connected counter: "
				<< config.encoder.not_connected_counter << modm::endl;
			config.foc.foc_state = Configuration::FocState::Disable;
			config.special_action = Configuration::SpecialActions::Abort;
			Board::Motor::setCompareValue(0);
			config.foc.voltage_integral_D=0;
			config.foc.voltage_integral_Q=0;
		}
		else if (safety_timeout.execute()){
			config.foc.foc_state = Configuration::FocState::Disable;
			config.foc.control_state = Configuration::ControlState::Disable;
			Board::Motor::setCompareValue(0);
			config.foc.voltage_integral_D=0;
			config.foc.voltage_integral_Q=0;
		}
		else if (safety_timeout.isArmed()){

		}


	}


	void
	initialize_gate_driver(bool calibrate_current_offset = true)
	{
		config.update_motor_parameters(mn4006Parameters);
		config.update_current_sense_parameters(wbPcb);
		config.update_control_loop_parameters(controlLoopParameters);

		Board::MotorBridge::GateDriverEnable::set();
		modm::Drv832xSpi<Board::MotorBridge::GateDriver::Spi, Board::MotorBridge::GateDriver::Cs> gateDriver;
		RF_CALL_BLOCKING(gateDriver.initialize());
		RF_CALL_BLOCKING(gateDriver.commit());

		Board::Motor::MotorTimer::start();
		Board::Encoder::As5047::Timer::start();

		Board::Motor::configure(Board::Motor::Phase::PhaseU, Board::Motor::PhaseConfig::Pwm);
		Board::Motor::configure(Board::Motor::Phase::PhaseV, Board::Motor::PhaseConfig::Pwm);
		Board::Motor::configure(Board::Motor::Phase::PhaseW, Board::Motor::PhaseConfig::Pwm);

		Board::Motor::setCompareValue(0);

		MODM_LOG_INFO << "Initialized Gate Driver" << modm::endl;

		if(calibrate_current_offset)
		{
			Board::MotorCurrent::calibrateOffset();
			MODM_LOG_INFO << "Calibrated current offsets are:  "
							<< "U : "
							<< Board::MotorCurrent::offset_U
							<< "        V : "
							<< Board::MotorCurrent::offset_V
							<< "        W : "
							<< Board::MotorCurrent::offset_W << modm::endl;
		}

		main_configuration.encoder.reset = true;
		librobots2::motor::setSvmOutputMagnitudeAngle<Board::Motor>(0.1, 0);
		modm::delay_ms(1000);
		main_configuration.persistent_data.phase_offset = main_configuration.encoder.position;
		MODM_LOG_DEBUG << "\nNaive encoder calibration would be :" << main_configuration.persistent_data.phase_offset << modm::endl << modm::endl;

		librobots2::motor::setSvmOutputMagnitudeAngle<Board::Motor>(0, 0);
		modm::delay_ms(100);
		MODM_LOG_DEBUG << "Compare value after librobots2::motor::setSvmOutputMagnitudeAngle<Board::Motor>(0, 0):  ";
		MODM_LOG_DEBUG << " U: " << Board::Motor::MotorTimer::getCompareValue(1);
		MODM_LOG_DEBUG << " V: " << Board::Motor::MotorTimer::getCompareValue(2);
		MODM_LOG_DEBUG << " W: " << Board::Motor::MotorTimer::getCompareValue(3);
		MODM_LOG_DEBUG << modm::endl;
	}


	void
	run_cogging_calibration(){
		config.special_action_progress = 0;
		MODM_LOG_INFO << "\nEntering cogging torque calibration" << modm::endl;
		const auto raw_encoder = main_configuration.encoder.raw_value.load(std::memory_order_relaxed);
		const auto starting_setpoint = raw_encoder & 0x3ffc;
		auto setpoint = starting_setpoint;
		constexpr size_t cogging_samples_per_position{100};
		constexpr size_t retries{10};
		float samples_buffer[cogging_samples_per_position];
		constexpr float variance_threshold{0.2};
		constexpr auto settle_time{std::chrono::milliseconds(50)};
		constexpr auto sample_time{std::chrono::microseconds(200)};
		constexpr std::array<int32_t, 2> dirs{1,-1};
		for ( auto& v : config.persistent_data.feedforward_cogging_torques) v = 0;
		for (const auto& dir : dirs)
		{
			for (size_t ii = 0; ii<4096; ii++)
			{
				config.foc.posctrl.setpoint = raw_encoder_to_electric_aligned_degrees(config, setpoint);
				MODM_LOG_INFO.flush();
				config.foc.control_state = Configuration::ControlState::PositionControl;
				config.foc.foc_state = Configuration::FocState::CurrentControl;
				safety_timeout.restart();
				modm::fiber::sleep(settle_time);
				for (size_t rr = 0; rr < retries; rr++)
				{
					float mean{0};
					for (size_t jj = 0; jj<cogging_samples_per_position; jj++)
					{
						if (config.special_action == Configuration::SpecialActions::Abort) return;
						config.foc.control_state = Configuration::ControlState::PositionControl;
						config.foc.foc_state = Configuration::FocState::CurrentControl;
						safety_timeout.restart();
						samples_buffer[jj] = config.foc.commanded_torque.load(std::memory_order_relaxed);
						modm::fiber::sleep(sample_time);
						mean += samples_buffer[jj]/cogging_samples_per_position;
					}
					float variance{0};
					for (size_t jj = 0; jj<cogging_samples_per_position; jj++)
						variance += std::pow(samples_buffer[jj]-mean, 2)/cogging_samples_per_position;

					if (variance < variance_threshold){
						if ((std::abs(mean) > std::abs(config.persistent_data.feedforward_cogging_torques[size_t{setpoint/4}])))
							config.persistent_data.feedforward_cogging_torques[size_t{setpoint/4}] = mean;
						break;
					}
					else if(rr == retries-1){
						MODM_LOG_INFO << "Cogging torque calibration failed in step " << ii << " after " << rr << " retries." << modm::endl;
						MODM_LOG_INFO << "Variance was " << variance << modm::endl;
						config.print_encoder();
						config.special_action = Configuration::SpecialActions::CoggingCalibrationFail;
						return;
					}
				}
				if (ii%8==0){
					MODM_LOG_INFO << "Cogging torque calibration for dir "<< dir << " at " << int{ii*100/4096} << " percent, current setpoint " << config.foc.posctrl.setpoint << modm::endl;
					config.special_action_progress = uint8_t{ii*0xff/4096};
				}
				setpoint += dir*4;
				if ( setpoint >= 0x4000){
					setpoint -= 0x4000;
				}
				else if ( setpoint < 0){
					setpoint += 0x4000;
				}

			}
		}

		save_to_flash();
		config.special_action = Configuration::SpecialActions::CoggingCalibrationSuccess;

	}

	void
	run_encoder_offset_calibration(){
		// reset the encoder
		main_configuration.encoder.reset = true;
		// modm::delay_ms(1); we should wait now, but locking takes long enough anyways

		// ramp up open loop current to lock rotor
		constexpr size_t ramp_up_counts{1000};
		constexpr float max_locking_current{0.1}; // fraction of max pwm we want to reach
		for(size_t i = 0; i<= ramp_up_counts; i++){
			float open_loop_current = max_locking_current * float{i}/float{ramp_up_counts};
			librobots2::motor::setSvmOutputMagnitudeAngle<Board::Motor>(open_loop_current, 0);
			modm::fiber::sleep(std::chrono::milliseconds(2));
		}

		// wait some time to let rotor come to standstill
		modm::delay(std::chrono::milliseconds(500));
		float electrical_angle{0}; // we are now at zero electrical angle
		const int initial_encoder_position = config.encoder.position;

		// move one electrical revolution left
		int64_t encoder_values_sum{0};
		constexpr float electrical_angle_step = 1; // step in degrees to move electrical angle
		constexpr std::array<int, 4> electrical_directions{1,-1,-1,1}; // we move right, left, left, right
		std::array<int64_t, electrical_directions.size()+1> encoder_values_sum_intermediate{initial_encoder_position,0,0,0,0};
		constexpr size_t num_steps{360};
		for (size_t j=0; j<electrical_directions.size(); j++){
			const auto electrical_direction_now = electrical_directions[j];
			for (size_t i = 0; i<num_steps; i++){
				encoder_values_sum += main_configuration.encoder.position;
				electrical_angle += (electrical_direction_now*electrical_angle_step);
				librobots2::motor::setSvmOutputMagnitudeAngle<Board::Motor>(max_locking_current, electrical_angle);
				modm::fiber::sleep(std::chrono::milliseconds(5));
			}
			encoder_values_sum_intermediate[j+1] = int64_t{encoder_values_sum};
		}
		Board::Motor::setCompareValue(0);

		// retrieve direction
		std::array<int,4> encoder_directions{0,0,0,0};
		for (size_t j=1; j<electrical_directions.size(); j++)
		{
			const auto delta_encoder = (encoder_values_sum_intermediate[j]-encoder_values_sum_intermediate[j-1])/num_steps;
			encoder_directions[j-1] = delta_encoder > 0 ? 1 : -1;
		}

		// save offset
		config.persistent_data.inverted = !(encoder_directions[0] ==  electrical_directions[0]);
		config.persistent_data.phase_offset = encoder_values_sum / (electrical_directions.size()*num_steps);

		MODM_LOG_INFO << "\n Calibration results: " << modm::endl;
		MODM_LOG_INFO << "\t encoder offset = " << config.persistent_data.phase_offset << modm::endl;
		MODM_LOG_INFO << "\t encoder inverted = " << config.persistent_data.inverted << modm::endl;

		save_to_flash();
	}

	modm::PreciseTimeout safety_timeout;

private:
	Configuration &config;
};

#endif // CONTROL_THREAD_HPP
