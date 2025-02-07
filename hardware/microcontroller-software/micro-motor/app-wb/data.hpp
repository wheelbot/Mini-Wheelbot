/* data.hpp
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

#ifndef DATA_HPP
#define DATA_HPP

#include "utils.hpp"
#include <micro-motor/hardware.hpp>
#include <modm/processing/fiber.hpp>
#include <atomic>

extern "C" const uint32_t __flash_reserved_start[];
constexpr auto max_flash_pages{256};

struct Configuration{

	// debug output mode
	enum class DebugMode{
		Disable,
		Default,
		ScopeIdq,
		ScopePosIq,
	};
	DebugMode dbgmode{DebugMode::Default};
	std::chrono::milliseconds dbgtime{1000};

	// foc control mode
	enum class FocState{
		Disable,
		CurrentControl,
	};

	// higher level controller
	enum class ControlState{
		Disable,
		PositionControl,
	};

	// special action states
	enum class SpecialActions : uint8_t {
		None						= 0,
		EncoderCalibrationStart		= 1,
		EncoderCalibrationRunning	= 2,
		EncoderCalibrationSuccess	= 3,
		EncoderCalibrationFail		= 4,
		CoggingCalibrationStart		= 5,
		CoggingCalibrationRunning	= 6,
		CoggingCalibrationSuccess	= 7,
		CoggingCalibrationFail		= 8,
		Abort						= 9,
	};
	SpecialActions special_action{SpecialActions::None};
	uint8_t special_action_progress{0};

	// parameters of the higher level position controller (ControlState::PositionControl), used inside interrupt!
	struct PositionControl{
		std::atomic<float> setpoint{0}; // [deg] position setpoint with same origin as angle_degrees_aligned_with_electrical_frame
		std::atomic<float> integral_torque{0}; // torque integral [A], no windup protection!
		std::atomic<float> last_err{0}; // [deg] last position error
		std::atomic<float> P_gain{1};
		std::atomic<float> D_gain{0};
		std::atomic<float> I_gain{2};
	};

	// parameters for foc control, used insied interrupt!
	struct Foc {
		std::atomic<float> current_D_out{0}; // [A] last measured D frame current for debugging
		std::atomic<float> current_Q_out{0}; // [A] last measured Q frame current for debugging
		std::atomic<float> current_U{0}; // [A] last measured U phase current
		std::atomic<float> current_V{0}; // [A] last measured V phase current
		std::atomic<float> current_W{0}; // [A] last measured W phase current
		std::atomic<float> voltage_Vin{0}; // [V] last measured input bus voltage
		std::atomic<float> commanded_torque{0}; // [A] commanded Q frame current
		FocState foc_state{FocState::Disable}; // current foc state
		ControlState control_state{ControlState::Disable}; // current higher level controller state
		PositionControl posctrl; // current parameters for higher level controller
		std::atomic<float> phase_amp_per_adc_reading; // configurable conversion from adc reading to phase amp
		std::atomic<float> input_voltage_per_adc_reading; // configurable conversion from adc reading to input voltage
		std::atomic<float> P_gain{0}; // foc D and Q frame PI controller gains, these are be automatically computed
		std::atomic<float> I_gain{0}; // foc D and Q frame PI controller gains, these are be automatically computed
		std::atomic<float> loop_time{1e-6}; // time between two executions of foc interrupt
		std::atomic<float> voltage_integral_D{0}; // [V] integral part of foc current control
		std::atomic<float> voltage_integral_Q{0}; // [V] integral part of foc current control
	};
	Foc foc;

	/// @brief Tuning PI controller for bldc foc
	// https://e2e.ti.com/blogs_/b/industrial_strength/posts/teaching-your-pi-controller-to-behave-part-ii
	struct BldcMotorParameters{
		float inductance;                           // [H] equivalent DC inductance
		float resistance;                           // [Ohm] equivalent DC resistance
		float electrical_per_mechanical_rev;    	// []
	};

	struct CurrentSenseParameters{
		float reference_voltage;            // [V]
		float amp_gain;                     // [V/V]
		float adc_resolution;               // []
		float shunt_resistance;             // [Ohm]
	};

	struct ControlLoopParameters{
		float current_control_loop_rate;    // [Hz]
		float bandwidth;
	};

	BldcMotorParameters bldc_motor_parameters;
	ControlLoopParameters control_loop_parameters;
	CurrentSenseParameters current_sense_parameters;

    struct Encoder{
        uint16_t resolution = 0x3FFF+1; // [ticks/mechanicalrevolution] total number of encoder ticks per revolution
		uint16_t inbuf{0}; // memory used to transfer spi buffer, do not access outside of encoder interrupt!!!
        std::atomic<int32_t> raw_value{0}; // [ticks] raw transferred encoder value
        std::atomic<int32_t> last_raw_value{0}; // [ticks] previous raw transferred encoder value
		std::atomic<int32_t> position; // [ticks] absolute position (multi-turn), beware, overflow of int32_t would occur after approx 8 min of 8k rpm ;-)
		// std::atomic<float> phase_offset_float{0}; // [degrees]
        std::atomic<float> angle_degrees_aligned_with_electrical_frame{0}; // [degrees]
		std::atomic<float> degrees_per_tick{360.f/(0x3FFF+1)};
		std::atomic<size_t> corrupted_data_counter{0}; // counter incremented every time parity doesnt match, deacreased otherwise
		std::atomic<size_t> not_connected_counter{0}; // counter increased every time all zeros or all ones is received, decreased otherwise
		bool reset{true}; // flag used for resetting the encoder (absolute position)
    };
    Encoder encoder;

	struct PersistentData{
		uint8_t auto_reload{0}; // [true/false] first byte of persistent config, determines if rest of this struct is automatically loaded from flash at boot
		std::array<float, 4096> feedforward_cogging_torques{1.234,3,4540,6,7,78,0,0,0,0,0}; // [A] struct holding feedfoward compensation table for cogging, encoder resolution should be integer multiple of this
		int16_t phase_offset{0}; // [ticks] offset between electrical and encoder
		bool inverted{false}; // [true/false] if electrical is same as encoder direction
		std::optional<uint8_t> board_id{std::nullopt}; // [1...7]
		bool use_cogging_compensation{false};
	};
	PersistentData persistent_data;

	void update_motor_parameters(const BldcMotorParameters& new_params){
		bldc_motor_parameters = new_params;

	};

	void update_control_loop_parameters(const ControlLoopParameters& new_params){
		control_loop_parameters = new_params;
		foc.loop_time = 1.f/control_loop_parameters.current_control_loop_rate;
		const float p_gain = control_loop_parameters.bandwidth * bldc_motor_parameters.inductance;
		const float plant_pole = bldc_motor_parameters.resistance / bldc_motor_parameters.inductance;
		foc.P_gain = p_gain;
		foc.I_gain = plant_pole * p_gain;
	};

	void update_current_sense_parameters(const CurrentSenseParameters& new_params){
		current_sense_parameters = new_params;
		foc.phase_amp_per_adc_reading =
			current_sense_parameters.reference_voltage / (
				current_sense_parameters.shunt_resistance * current_sense_parameters.amp_gain * current_sense_parameters.adc_resolution
			);
		foc.input_voltage_per_adc_reading =
			current_sense_parameters.reference_voltage /current_sense_parameters.adc_resolution * 21.f;
	};

	void
	print_encoder()
	{
		MODM_LOG_INFO << "Encoder offset is: " << persistent_data.phase_offset
			<< "   inverted: " << persistent_data.inverted
			<< modm::endl;

		MODM_LOG_INFO << "Cogging calibration is: ";
		for (const auto& f : persistent_data.feedforward_cogging_torques){
			MODM_LOG_INFO.printf("%.3f, ", f);
		}
		MODM_LOG_INFO << modm::endl;
		MODM_LOG_INFO.flush();
	}
};

static Configuration main_configuration{};

void save_to_flash(){
		MODM_LOG_INFO << "Saving data to flash" << modm::endl;
		main_configuration.print_encoder();

		uint32_t err{0};
		const size_t page_start = Flash::getPage(reinterpret_cast<uint32_t>(&__flash_reserved_start));
		const size_t num_bytes = sizeof(Configuration::PersistentData);
		const size_t flash_page_size = Flash::getSize(page_start);
		const size_t end_page = page_start + (num_bytes + flash_page_size - 1) / flash_page_size;
		if (end_page >= max_flash_pages)
		{
			MODM_LOG_ERROR << "\nRequested flash end page exceeds flash [" << page_start << ", "
						   << end_page << ")" << modm::endl;
			MODM_LOG_INFO.flush();
			while (1);
		}

		// erase the pages before programming
		MODM_LOG_INFO << "\nErasing flash sectors [" << page_start << ", " << end_page << ")"
					  << modm::endl;
		MODM_LOG_INFO.flush();
		if (not Flash::unlock())
		{
			MODM_LOG_INFO << "Flash unlock failed!" << modm::endl;
			while (1);
		}
		for (size_t page{page_start}; page < end_page; page++) err |= Flash::erase(page);
		if (err != 0)
		{
			MODM_LOG_ERROR << "\nThere was an error while erasing flash!" << modm::endl;
			MODM_LOG_INFO.flush();
			while (1);
		}

		// now, write the padded data
		MODM_LOG_INFO << "\nWriting, word size: " << sizeof(Flash::MaxWordType) << "...";

		auto flash_write_base_addr{reinterpret_cast<uint32_t>(Flash::getAddr(page_start))};
		for (size_t ii = 0; ii < sizeof(Configuration::PersistentData); ii += sizeof(Flash::MaxWordType))
		{
			Flash::MaxWordType outdata;
			memcpy(&outdata, reinterpret_cast<char*>(&main_configuration.persistent_data)+ii, sizeof(Flash::MaxWordType));
			err |= Flash::program(flash_write_base_addr + ii, outdata);
		}
		if (err != 0)
		{
			MODM_LOG_ERROR << "\nThere was an error while programming flash!" << modm::endl;
			MODM_LOG_INFO.flush();
		}
		else
		{
			MODM_LOG_INFO << "Writing complete! " << modm::endl;
			MODM_LOG_INFO.flush();
		}
		return;
}

uint8_t check_auto_reload(){
	const size_t page_start = Flash::getPage(reinterpret_cast<uint32_t>(&__flash_reserved_start));
	return *Flash::getAddr(page_start);
}

void load_from_flash(){
		const size_t page_start = Flash::getPage(reinterpret_cast<uint32_t>(&__flash_reserved_start));
		auto flash_read_base_addr = reinterpret_cast<uint32_t>(Flash::getAddr(page_start));
		MODM_LOG_INFO.printf("\n FLASH READ ADDR: 0x%08lx\n", flash_read_base_addr);
		memcpy(&main_configuration.persistent_data, reinterpret_cast<void*>(flash_read_base_addr), sizeof(Configuration::PersistentData));
		MODM_LOG_INFO << "\n Successfully restored config from flash" << modm::endl;
		MODM_LOG_INFO.flush();
		main_configuration.print_encoder();
}

#endif // DATA_HPP
