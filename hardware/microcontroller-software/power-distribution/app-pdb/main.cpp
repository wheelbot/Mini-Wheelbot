#include <modm/platform.hpp>
#include <modm/debug/logger.hpp>
#include "board.hpp"

#include <modm/math/filter/pid.hpp>

using namespace Board;
using namespace modm::literals;


// modm::IODeviceWrapper< Board::Ui::DebugUart, modm::IOBuffer::BlockIfFull > loggerDevice;
modm::log::Logger modm::log::debug(loggerDevice);
modm::log::Logger modm::log::info(loggerDevice);
modm::log::Logger modm::log::warning(loggerDevice);
modm::log::Logger modm::log::error(loggerDevice);

using PidParameters = modm::Pid<float>::Parameter;
using Pid = modm::Pid<float>;
// auto braking_pid_parameters = PidParameters{0.005, 0.005, 0, 0.9/0.01, 0.9};
auto braking_pid_parameters = PidParameters{0.2, 0.001, 0, 0.9/0.01, 1};
auto braking_pid = Pid{braking_pid_parameters};

constexpr auto i_drive_max = 15.f; // [A]
constexpr auto v_supply_min = 15.f; // [V]

float braking_pid_value{0};

bool break_overwrite{0};

MODM_ISR(TIM8_UP)
{
	using Timer = Board::DrivePower::BrakingTimer;
	Timer::acknowledgeInterruptFlags(Timer::InterruptFlag::Update);

	float v_supply = Adc1::getValue() * DrivePower::v_supply_div;
	float i_drive  = (Adc2::getValue() - 0x7ff) * DrivePower::i_drive_div;

	// if (i_drive >= i_drive_max){
	// 	// trigger estop!!!
	// 	DrivePower::disableDrive();
	// 	LogicPower::EnableMotor1::reset();
	// 	LogicPower::EnableMotor2::reset();
	// }

	if (v_supply <= v_supply_min){
		// trigger estop!!!
		DrivePower::disableDrive();
		LogicPower::EnableMotor1::reset();
		LogicPower::EnableMotor2::reset();

	}

	braking_pid.update(v_supply-DrivePower::v_supply_max);
	braking_pid_value = braking_pid.getValue();

	if(break_overwrite){
		DrivePower::setBrakingCompareValue(DrivePower::MaxPwm);
	}
	else if ( braking_pid_value > 0 ) {
		uint16_t output = braking_pid_value*DrivePower::MaxPwm;
		DrivePower::setBrakingCompareValue(output);
	}
	else {
		DrivePower::setBrakingCompareValue(0);
	}

}


class UiBuzzer{
public:
	modm::ShortTimeout timeout;
	enum class State{
		On, Off
	};

	State state{State::Off};

public:
	void
	update(){
		if(timeout.execute() && (state==State::On)){
			Ui::buzzerOff();
			state=State::Off;
		}
	}

	template< typename Rep, typename Period >
	void
	beep(std::chrono::duration<Rep, Period> interval){
		Ui::buzzerOn();
		state=State::On;
		timeout.restart(interval);
	}

};



class EStop{
public:
	enum struct State{
		Stop = 0,
		Arm = 1,
		Running = 2
	};

	modm::ShortPeriodicTimer timeout;
	State current_state{State::Arm};

	UiBuzzer buzzer;

	template< typename Rep, typename Period >
	EStop(std::chrono::duration<Rep, Period> interval)
		: timeout(modm::ShortPeriodicTimer{interval})
	{};

	void executeStop(){
		current_state = State::Stop;
		DrivePower::disableDrive();
		LogicPower::EnableMotor1::reset();
		LogicPower::EnableMotor2::reset();
	}

	void executeArmToRunning(){
		current_state = State::Running;
		DrivePower::enableDrive();
		LogicPower::EnableMotor1::set();
		LogicPower::EnableMotor2::set();
		buzzer.beep(500ms);
	}

	void executeStopToArm(){
		current_state = State::Arm;
	}

	void update(){
		buzzer.update();
		if (timeout.execute()){
			if (current_state==State::Running || current_state == State::Arm){
				executeStop();
			}
			DrivePower::disableDrive();
		}
		if (Nrf::Data::Packet packet; Nrf::Data::getPacket(packet))
		{
			if (static_cast<State>(packet.payload[0]) == State::Stop){
				executeStop();
			}
			else if (current_state == State::Arm && static_cast<State>(packet.payload[0]) == State::Running){
				executeArmToRunning();
			}
			else if( ( current_state == State::Stop ) && static_cast<State>(packet.payload[0]) == State::Arm){
				executeStopToArm();
			}

			// if (current_state != State::Stop){
			timeout.restart();
			// }

			// MODM_LOG_INFO << "Receiving packet " << packet.payload[0] << " from " << packet.getSource() << modm::endl;

		}
	}

	inline
	State getCurrentState(){
		return current_state;
	}
};


int
main()
{
	Board::initialize();
	Ui::LedRed::setOutput(false);
	Ui::LedGreen::setOutput(true);

	// Use the logging streams to print some messages.
	// Change MODM_LOG_LEVEL above to enable or disable these messages
	MODM_LOG_DEBUG   << "debug"   << modm::endl;
	MODM_LOG_INFO    << "info"    << modm::endl;
	MODM_LOG_WARNING << "warning" << modm::endl;
	MODM_LOG_ERROR   << "error"   << modm::endl;

	// LogicPower::enableAll();
	LogicPower::EnablePi::set();
	DrivePower::disableDrive();

	for (int i = 0; i<20; i++)
	{
		Ui::buzzerOn();
		modm::delay(50ms);
		Ui::buzzerOff();
		modm::delay(50ms);
	}

	// modm::delay(2000ms);
	Board::DrivePower::setBrakingCompareValue(0);
	// DrivePower::enableDrive();


	modm::ShortPeriodicTimer debug(100ms);

	std::array<uint8_t,Nrf::payload_length> payload{0};

	// EStop estop(300ms);

	DrivePower::enableDrive();
	LogicPower::EnableMotor1::set();
	LogicPower::EnableMotor2::set();

	while (true)
	{
		// estop.update();
		// estop.executeArmToRunning();
		Ui::buzzerOff();

		if(debug.execute()){
			// Ui::buzzerOff();
			Ui::LedRed::toggle();
			// Adc1::startConversion();
			// Adc2::startConversion();
					// Adc2::startConversion();
			// while(!Adc1::isConversionFinished())
			// 	;
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
			MODM_LOG_INFO << " brakingpid=";
			MODM_LOG_INFO.printf("%.3f", braking_pid_value);
			MODM_LOG_INFO << modm::endl;

			// DrivePower::EnableBrakingH::toggle();

			// modm::delay(500ms);
			// MODM_LOG_INFO    << "VDrive: " << Adc1::getValue() * 2.9 / 0xfff * 11.f << modm::endl;
			// MODM_LOG_INFO    << "CurrentDrive: " << Adc2::getValue() << modm::endl;

			// MODM_LOG_INFO << Nrf::Phy::readStatus() << modm::endl;
			// MODM_LOG_INFO << (uint8_t)estop.getCurrentState() << modm::endl;
		}
	}

	return 0;
}
