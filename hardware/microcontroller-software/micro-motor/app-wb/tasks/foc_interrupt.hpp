/* foc_interrupt.hpp
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

#ifndef FOC_INTERRUPT_HPP
#define FOC_INTERRUPT_HPP

#include <modm/platform.hpp>
#include <micro-motor/hardware.hpp>
#include "../data.hpp"

MODM_ISR(TIM1_UP_TIM16)
{
	Timer1::acknowledgeInterruptFlags(Timer1::InterruptFlag::Update);

	// for timing of motor control loop on encoder SPI
	// Board::Encoder::As5047::Cs::reset();

	// retrieve latest current and voltage measurements
	main_configuration.foc.current_U   = int16_t{Board::MotorCurrent::AdcU::getValue()} - 0x7ff - Board::MotorCurrent::offset_U;
	main_configuration.foc.current_V   = int16_t{Board::MotorCurrent::AdcV::getValue()} - 0x7ff - Board::MotorCurrent::offset_V;
	main_configuration.foc.current_W   = int16_t{Board::MotorCurrent::AdcW::getValue()} - 0x7ff - Board::MotorCurrent::offset_W;
	main_configuration.foc.voltage_Vin = int16_t{Board::MotorCurrent::AdcIn::getValue()};

	// Transform to SI units, [A] and [V]
	main_configuration.foc.current_U   =  main_configuration.foc.current_U * main_configuration.foc.phase_amp_per_adc_reading;
	main_configuration.foc.current_V   =  main_configuration.foc.current_V * main_configuration.foc.phase_amp_per_adc_reading;
	main_configuration.foc.current_W   =  main_configuration.foc.current_W * main_configuration.foc.phase_amp_per_adc_reading;
	main_configuration.foc.voltage_Vin =  main_configuration.foc.voltage_Vin * main_configuration.foc.input_voltage_per_adc_reading;

	// TODO: CHECK OVERCURRENT / OVEROLTAGE PANIC HERE !!!

	if (main_configuration.foc.foc_state == Configuration::FocState::CurrentControl){
		float angle_degrees = main_configuration.encoder.angle_degrees_aligned_with_electrical_frame;
		while(angle_degrees>180.f) angle_degrees-=360.f;
		while(angle_degrees<-180.f) angle_degrees+=360.f;

		if (main_configuration.foc.control_state == Configuration::ControlState::PositionControl){
			float position_error = main_configuration.foc.posctrl.setpoint - angle_degrees;
			while(position_error>180.f) position_error-=360.f;
			while(position_error<-180.f) position_error+=360.f;

			float commanded_torque =
				main_configuration.foc.posctrl.P_gain * position_error
				+ main_configuration.foc.posctrl.integral_torque;

			if (commanded_torque >  5.f) commanded_torque =  5.f;
			if (commanded_torque < -5.f) commanded_torque = -5.f;
			main_configuration.foc.commanded_torque = commanded_torque;

			float integral_torque = main_configuration.foc.posctrl.integral_torque + main_configuration.foc.posctrl.I_gain*main_configuration.foc.loop_time*position_error;
			if (integral_torque >  0.5f) integral_torque =  0.5f;
			if (integral_torque < -0.5f) integral_torque = -0.5f;
			main_configuration.foc.posctrl.integral_torque = integral_torque;
		}

		// we use all three phase current measurements in Clarke Transform
		// Odrive only uses two, MESC uses two phases with lowest duty cycle, unit is [A]
		const auto [current_A, current_B] = librobots2::motor::clarkeTransform(main_configuration.foc.current_U, main_configuration.foc.current_V, main_configuration.foc.current_W);

		// get electrical angle from mechanical, unit is [deg]
		const float electrical_angle_degrees = angle_degrees * main_configuration.bldc_motor_parameters.electrical_per_mechanical_rev;

		// transform current to direct and quadrature frame, unit is [A]
		// const auto [current_D, current_Q] = librobots2::motor::parkTransform(current_A, current_B, electrical_angle_degrees);
		float sine{}, cosine{};
		arm_sin_cos_f32(electrical_angle_degrees, &sine, &cosine);
		const auto current_D =  cosine * current_A +   sine * current_B;
		const auto current_Q =   -sine * current_A + cosine * current_B;

		main_configuration.foc.current_D_out = current_D;
		main_configuration.foc.current_Q_out = current_Q;

		// calculate feedforward terms
		// TODO: Odrive Firmware R_wL_FF_enable, bEMF_FF_enable
		const float feedforward_voltage_D{0};
		float feedforward_voltage_Q{0};

		// run PI controller, I find modm's PI complicated, so we don't use it
		const float current_D_setpoint{0};
		float current_Q_setpoint = main_configuration.foc.commanded_torque;
		if (main_configuration.persistent_data.use_cogging_compensation)
		{
			const auto raw_encoder = main_configuration.encoder.raw_value.load(std::memory_order_relaxed);
			const auto idx_past = raw_encoder / 4;
			const auto idx_next = idx_past+1 < 4096 ? idx_past+1 : idx_past+1-4096;

			constexpr auto cogging_correction{1.3f};
			const auto feedforward_magnitude_past = main_configuration.persistent_data.feedforward_cogging_torques[idx_past];
			const auto feedforward_magnitude_next = main_configuration.persistent_data.feedforward_cogging_torques[idx_next];
			const auto factor = float{(raw_encoder & 0x3)}/4.f;
			auto feedforward_magnitude = (1.f-factor)*feedforward_magnitude_past + factor*feedforward_magnitude_next;
			feedforward_magnitude *= cogging_correction;
			if ( (main_configuration.foc.commanded_torque > 0) && (feedforward_magnitude >0) ){
				current_Q_setpoint += feedforward_magnitude;
			}
			else if ( (main_configuration.foc.commanded_torque > 0) && (feedforward_magnitude < 0) ){
				current_Q_setpoint -= feedforward_magnitude;
			}
			else if ( (main_configuration.foc.commanded_torque < 0) && (feedforward_magnitude > 0) ){
				current_Q_setpoint -= feedforward_magnitude;
			}
			else if ( (main_configuration.foc.commanded_torque < 0) && (feedforward_magnitude < 0) ){
				current_Q_setpoint += feedforward_magnitude;
			}

		}

		const float current_D_err = current_D_setpoint - current_D;
		const float current_Q_err = current_Q_setpoint - current_Q;

		const float modulation_to_voltage = (2.f/3.f)*main_configuration.foc.voltage_Vin;
		const float voltage_to_modulation = 1.f/modulation_to_voltage;

		float modulation_D = voltage_to_modulation *
			(
				feedforward_voltage_D
				+ main_configuration.foc.voltage_integral_D
				+ current_D_err * main_configuration.foc.P_gain
			);

		float modulation_Q = voltage_to_modulation *
			(
				feedforward_voltage_Q
				+ main_configuration.foc.voltage_integral_Q
				+ current_Q_err * main_configuration.foc.P_gain
			);

		float modulation_norm{};
		arm_sqrt_f32(modulation_D * modulation_D + modulation_Q * modulation_Q, &modulation_norm);
		const float modulation_scalefactor = 0.80f * std::numbers::sqrt3_v<float> / 2.f * 1.0f / modulation_norm;

		if (modulation_scalefactor < 1.0f)
		{
            modulation_D *= modulation_scalefactor;
            modulation_Q *= modulation_scalefactor;
        	main_configuration.foc.voltage_integral_D = main_configuration.foc.voltage_integral_D*0.99f;
        	main_configuration.foc.voltage_integral_Q = main_configuration.foc.voltage_integral_Q*0.99f;
		}
		else
		{
			main_configuration.foc.voltage_integral_D  += current_D_err * ( main_configuration.foc.I_gain * main_configuration.foc.loop_time);
			main_configuration.foc.voltage_integral_Q  += current_Q_err * ( main_configuration.foc.I_gain * main_configuration.foc.loop_time);
		}

		const auto voltage_A  = cosine * modulation_D -   sine * modulation_Q;
    	const auto voltage_B  =   sine * modulation_D + cosine * modulation_Q;
    	librobots2::motor::setSvmOutput<Board::Motor>(voltage_A, voltage_B);
	}
}

#endif // FOC_INTERRUPT_HPP
