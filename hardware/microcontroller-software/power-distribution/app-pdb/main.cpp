// #include <modm/platform.hpp>
// #include <modm/debug/logger.hpp>
// #include <modm/processing/fiber/scheduler.hpp>
// #include <modm/processing/fiber.hpp>
#include "modm/platform.hpp"
#include "modm/debug/logger.hpp"
#include "modm/processing/fiber/scheduler.hpp"
#include "modm/processing/fiber.hpp"
#include "board.hpp"
#include "data.hpp"
#include "tasks/can_thread.hpp"
#include "tasks/control_thread.hpp"
#include "tasks/pdb_thread.hpp"
#include "tasks/debug_thread.hpp"
#include "tasks/buzzer_thread.hpp"

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

std::atomic<float> last_motor_current;

MODM_ISR(TIM8_UP)
{
	using Timer = Board::DrivePower::BrakingTimer;
	Timer::acknowledgeInterruptFlags(Timer::InterruptFlag::Update);

	float v_supply = Adc1::getValue() * DrivePower::v_supply_div;
	float i_drive  = static_cast<float>(
		static_cast<int>(Adc2::getValue()) - 0x7ff)* DrivePower::i_drive_div;
	last_motor_current.store(i_drive, std::memory_order_relaxed);
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
		MODM_LOG_INFO.printf("V_supply < V_supply_min\n");
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

CanThread can_thread(main_configuration, can_thread.safety_timeout);
ControlThread control_thread(main_configuration);
PDBThread pdb_thread(main_configuration);
DebugThread debug_thread(main_configuration);
BuzzerThread buzzer_thread(main_configuration);


extern modm::Fiber<2048> fiber_can_thread_run;

modm_faststack modm::Fiber<2048> fiber_can_thread_run([](){
	can_thread.initialize();
	while(1){
		can_thread.run();
		modm::this_fiber::yield();
	}
	}, modm::fiber::Start::Later);


modm_faststack modm::Fiber<2048> fiber_control_thread_run([](){
	while(1){
		control_thread.run();
		modm::this_fiber::yield();
	}
}, modm::fiber::Start::Later);

modm_faststack modm::Fiber<2048> fiber_pdb_thread_run([](){
	pdb_thread.initialize();
	while(1){
		pdb_thread.run();
		modm::this_fiber::yield();
	}
}, modm::fiber::Start::Later);

modm_faststack modm::Fiber<2048> fiber_debug_thread_run([](){
	while(1){
		debug_thread.run();
		float i_drive  = last_motor_current.load(std::memory_order_relaxed);
		MODM_LOG_INFO << "Measured drive current: " << int{i_drive*1000} << " mA" << modm::endl;
		modm::this_fiber::sleep_for(100ms);
		modm::this_fiber::yield();
	}
}, modm::fiber::Start::Later);

modm_faststack modm::Fiber<2048> fiber_buzzer_thread_run([](){
	while(1){
		buzzer_thread.run();
		modm::this_fiber::yield();
	}
	}, modm::fiber::Start::Later);

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

void COMP1_2_3_IRQHandler(void)
{
    // Clear pending EXTI flag
    EXTI->PR1 = (1U << 21);

	DrivePower::disableDrive();

	main_configuration.buzzer_action = Configuration::BuzzerActions::buzzingShortCircuitBreak;
}

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

	// modm::delay(2000ms);
	Board::DrivePower::setBrakingCompareValue(0);
	// DrivePower::enableDrive();


	modm::ShortPeriodicTimer debug(100ms);

	std::array<uint8_t,Nrf::payload_length> payload{0};

	// fiber_can_thread_run.watermark_stack();
	// fiber_debug_thread_run.watermark_stack();
	// fiber_control_thread_run.watermark_stack();
	// fiber_pdb_thread_run.watermark_stack();
	// fiber_buzzer_thread_run.watermark_stack();

	fiber_can_thread_run.start();
	fiber_control_thread_run.start();
	fiber_pdb_thread_run.start();
	fiber_debug_thread_run.start();
	fiber_buzzer_thread_run.start();


	modm::fiber::Scheduler::run();

	while (true)
	{
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
