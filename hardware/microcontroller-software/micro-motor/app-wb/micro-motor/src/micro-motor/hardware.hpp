/*
 * Copyright (C) 2019 Raphael Lehmann <raphael@rleh.de>
 * Copyright (C) 2021 Christopher Durand
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

#ifndef HARDWARE_V2_HPP
#define HARDWARE_V2_HPP

#include <modm/platform.hpp>
#include <modm/architecture/interface/clock.hpp>
#include <modm/platform/uart/uart_1.hpp>
#include <modm/math/filter/moving_average.hpp>

#include <librobots2/motor/motor_bridge.hpp>


using namespace modm::platform;

namespace Board
{
	using namespace modm::literals;

/// STM32G474RE running at 170MHz generated from the internal 16MHz crystal
// Dummy clock for devices
struct SystemClock {
	static constexpr uint32_t Frequency = 170_MHz;
	static constexpr uint32_t Ahb1      = Frequency;
	static constexpr uint32_t Ahb2      = Frequency;
	static constexpr uint32_t Apb1      = Frequency;
	static constexpr uint32_t Apb2      = Frequency;

	static constexpr uint32_t Cordic    = Ahb1;
	static constexpr uint32_t Crc       = Ahb1;
	static constexpr uint32_t Dma       = Ahb1;
	static constexpr uint32_t Dma1      = Dma;
	static constexpr uint32_t Dma2      = Dma;
	static constexpr uint32_t DmaMux    = Dma;
	static constexpr uint32_t Fmac      = Ahb1;

	static constexpr uint32_t Adc       = Ahb2;
	static constexpr uint32_t Adc1      = Adc;
	static constexpr uint32_t Adc2      = Adc;
	static constexpr uint32_t Adc3      = Adc;
	static constexpr uint32_t Adc4      = Adc;
	static constexpr uint32_t Adc5      = Adc;
	static constexpr uint32_t Dac       = Ahb2;
	static constexpr uint32_t Dac1      = Dac;
	static constexpr uint32_t Dac2      = Dac;
	static constexpr uint32_t Dac3      = Dac;
	static constexpr uint32_t Dac4      = Dac;
	static constexpr uint32_t Rng       = Ahb2;

	static constexpr uint32_t Can       = Apb1;
	static constexpr uint32_t Fdcan1    = Can;
	static constexpr uint32_t Fdcan2    = Can;
	static constexpr uint32_t Fdcan3    = Can;
	static constexpr uint32_t I2c       = Apb1;
	static constexpr uint32_t I2c1      = I2c;
	static constexpr uint32_t I2c2      = I2c;
	static constexpr uint32_t I2c3      = I2c;
	static constexpr uint32_t I2c4      = I2c;
	static constexpr uint32_t Lptim     = Apb1;
	static constexpr uint32_t Lpuart    = Apb1;
	static constexpr uint32_t Rtc       = Apb1;
	static constexpr uint32_t Spi2      = Apb1;
	static constexpr uint32_t Spi3      = Apb1;
	static constexpr uint32_t Uart4     = Apb1;
	static constexpr uint32_t Uart5     = Apb1;
	static constexpr uint32_t Usart2    = Apb1;
	static constexpr uint32_t Usart3    = Apb1;
	static constexpr uint32_t Usb       = Apb1;
	static constexpr uint32_t Apb1Timer = Apb1 * 1;
	static constexpr uint32_t Timer2    = Apb1Timer;
	static constexpr uint32_t Timer3    = Apb1Timer;
	static constexpr uint32_t Timer4    = Apb1Timer;
	static constexpr uint32_t Timer5    = Apb1Timer;
	static constexpr uint32_t Timer6    = Apb1Timer;
	static constexpr uint32_t Timer7    = Apb1Timer;

	static constexpr uint32_t Sai1      = Apb2;
	static constexpr uint32_t Spi1      = Apb2;
	static constexpr uint32_t Usart1    = Apb2;
	static constexpr uint32_t Apb2Timer = Apb2 * 1;
	static constexpr uint32_t Timer1    = Apb2Timer;
	static constexpr uint32_t Timer8    = Apb2Timer;
	static constexpr uint32_t Timer15   = Apb2Timer;
	static constexpr uint32_t Timer16   = Apb2Timer;
	static constexpr uint32_t Timer17   = Apb2Timer;
	static constexpr uint32_t Timer20   = Apb2Timer;

	static bool inline
	enable()
	{
		Rcc::enableInternalClock();	// 16MHz
		Rcc::PllFactors pllFactors{
			.pllM = 4,	//  16MHz / M= 4 ->   4MHz
			.pllN = 85,	//   4MHz * N=85 -> 340MHz
			.pllR = 2,	// 336MHz / R= 2 -> 170MHz = F_cpu
		};
		Rcc::enablePll(Rcc::PllSource::InternalClock, pllFactors);
		// set flash latency for 170MHz
		Rcc::setFlashLatency<Frequency>();
		// switch system clock to PLL output
		Rcc::enableSystemClock(Rcc::SystemClockSource::Pll);
		Rcc::setAhbPrescaler(Rcc::AhbPrescaler::Div1);
		// APB1 has max. 170MHz
		Rcc::setApb1Prescaler(Rcc::Apb1Prescaler::Div1);
		Rcc::setApb2Prescaler(Rcc::Apb2Prescaler::Div1);
		// update frequencies for busy-wait delay functions
		Rcc::updateCoreFrequency<Frequency>();

		Rcc::setCanClockSource(Rcc::CanClockSource::Pclk);

		return true;
	}
};

namespace Ui {
	using LedRed		= GpioB11;
	using LedGreen		= GpioB10;

	using DebugUartTx	= GpioB6;
	using DebugUartRx	= GpioB7;
	// using DebugUart		= Usart1;
	using DebugUart		= BufferedUart<UsartHal1, UartTxBuffer<2048>>;;
	static constexpr uint32_t DebugUartBaudrate = 460800_Bd;

	inline void
	initializeDebugUart()
	{
		DebugUart::connect<DebugUartTx::Tx, DebugUartRx::Rx>();
		DebugUart::initialize<SystemClock, DebugUartBaudrate>();
	}

	/**
	 * Should be used mutually exclusive to `initializeDac`
	 *
	 * Initializes the LEDs as standard outputs
	 */
	inline void
	initializeLeds()
	{
		LedRed::setOutput(false);
		LedGreen::setOutput(false);
	}

	// /**
	//  * Should be used mutually exclusive to `initializeLeds`
	//  *
	//  * Initializes the LEDs using the DAC
	//  */
	// inline void
	// initializeDac()
	// {
	// 	Dac1::connect<LedRed::Out1, LedGreen::Out2>();
	// 	Dac1::initialize<Board::SystemClock>();
	// 	Dac1::setMode(Dac1::Channel::Channel1, Dac1::Mode::ExternalWithBuffer);
	// 	Dac1::setMode(Dac1::Channel::Channel2, Dac1::Mode::ExternalWithBuffer);
	// 	Dac1::enableChannel(Dac1::Channel::Channel1);
	// 	Dac1::enableChannel(Dac1::Channel::Channel2);
	// }
}

struct Motor {
	using PhaseUN		= GpioB13;
	using PhaseUP		= GpioA8;
	using PhaseVN		= GpioB14;
	using PhaseVP		= GpioA9;
	using PhaseWN		= GpioB15;
	using PhaseWP		= GpioA10;

	using MotorTimer	= Timer1;

	using Phase = librobots2::motor::Phase;
	using PhaseConfig = librobots2::motor::PhaseConfig;
	using BridgeConfig = librobots2::motor::BridgeConfig;

	static constexpr uint16_t MaxPwm{2047}; // 11 bit PWM

	static inline void
	setCompareValue(uint16_t compareValue)
	{
		MotorTimer::setCompareValue(1, compareValue);
		MotorTimer::setCompareValue(2, compareValue);
		MotorTimer::setCompareValue(3, compareValue);
	}

	static inline void
	setCompareValue(Phase phase, uint16_t compareValue)
	{
		if (phase == Phase::PhaseU) {
			MotorTimer::setCompareValue(1, compareValue);
		} else if (phase == Phase::PhaseV) {
			MotorTimer::setCompareValue(2, compareValue);
		} else {
			MotorTimer::setCompareValue(3, compareValue);
		}
	}

	static void
	configure(Phase phase, PhaseConfig phaseOutputConfig)
	{
		switch(phaseOutputConfig) {
			case PhaseConfig::HiZ:
				MotorTimer::configureOutputChannel(static_cast<uint32_t>(phase) + 1,
						MotorTimer::OutputCompareMode::ForceActive,
						MotorTimer::PinState::Enable,
						MotorTimer::OutputComparePolarity::ActiveLow,
						MotorTimer::PinState::Enable,
						MotorTimer::OutputComparePolarity::ActiveHigh,
						MotorTimer::OutputComparePreload::Disable
						);
				break;
			case PhaseConfig::Pwm:
				MotorTimer::configureOutputChannel(static_cast<uint32_t>(phase) + 1,
						MotorTimer::OutputCompareMode::Pwm,
						MotorTimer::PinState::Enable,
						MotorTimer::OutputComparePolarity::ActiveHigh,
						MotorTimer::PinState::Enable,
						MotorTimer::OutputComparePolarity::ActiveHigh,
						MotorTimer::OutputComparePreload::Disable
						);
				break;
			case PhaseConfig::High:
				MotorTimer::configureOutputChannel(static_cast<uint32_t>(phase) + 1,
						MotorTimer::OutputCompareMode::ForceActive,
						MotorTimer::PinState::Enable,
						MotorTimer::OutputComparePolarity::ActiveHigh,
						MotorTimer::PinState::Enable,
						MotorTimer::OutputComparePolarity::ActiveHigh,
						MotorTimer::OutputComparePreload::Disable
						);
				break;
			case PhaseConfig::Low:
				MotorTimer::configureOutputChannel(static_cast<uint32_t>(phase) + 1,
						MotorTimer::OutputCompareMode::ForceActive,
						MotorTimer::PinState::Enable,
						MotorTimer::OutputComparePolarity::ActiveLow,
						MotorTimer::PinState::Enable,
						MotorTimer::OutputComparePolarity::ActiveLow,
						MotorTimer::OutputComparePreload::Disable
						);
				break;
		}
	}

	static inline void
	configure(const BridgeConfig& config)
	{
		configure(Phase::PhaseU, config.config[0]);
		configure(Phase::PhaseV, config.config[1]);
		configure(Phase::PhaseW, config.config[2]);
	}

	static inline void
	configure(PhaseConfig config)
	{
		configure(Phase::PhaseU, config);
		configure(Phase::PhaseV, config);
		configure(Phase::PhaseW, config);
	}

	// static inline void
	// initializeHall()
	// {
	// 	static_assert(HallPort::number_of_ports == 1, "Hall pins must be at the same port to guarantee atomic read operations.");

	// 	// Timer is not used for commutation
	// 	// Bldc motor commutation is done using external gpio pin interrupts
	// 	//HallPort::setInput(Gpio::InputType::PullUp);

	// 	HallU::setInput(Gpio::InputType::PullUp);
	// 	HallV::setInput(Gpio::InputType::PullUp);
	// 	HallW::setInput(Gpio::InputType::PullUp);

	// 	/*HallU::setInputTrigger(Gpio::InputTrigger::BothEdges);
	// 	HallU::enableExternalInterrupt();
	// 	HallU::enableExternalInterruptVector(HallInterruptPriority);
	// 	HallV::setInputTrigger(Gpio::InputTrigger::BothEdges);
	// 	HallV::enableExternalInterrupt();
	// 	HallV::enableExternalInterruptVector(HallInterruptPriority);
	// 	HallW::setInputTrigger(Gpio::InputTrigger::BothEdges);
	// 	HallW::enableExternalInterrupt();
	// 	HallW::enableExternalInterruptVector(HallInterruptPriority);*/
	// }

	static void
	initialize()
	{
		MotorTimer::enable();
		//MotorTimer::setMode(MotorTimer::Mode::UpCounter);
		MotorTimer::setMode(MotorTimer::Mode::CenterAligned1,
					   MotorTimer::SlaveMode::Disabled,
					   MotorTimer::SlaveModeTrigger::Internal0, // this timer is master
					   MotorTimer::MasterMode::Update); // trigger timer 4 on update event

		// MotorTimer clock: APB2 timer clock (170MHz)
		MotorTimer::setPrescaler(1);
		// Prescaler: 1 -> Timer counter frequency: 170MHz
		MotorTimer::setOverflow(MaxPwm);
		// Pwm frequency: 170MHz / 2048 / 2 = 83kHz

		configure(Phase::PhaseU, PhaseConfig::HiZ);
		configure(Phase::PhaseV, PhaseConfig::HiZ);
		configure(Phase::PhaseW, PhaseConfig::HiZ);

		setCompareValue(0);

		MotorTimer::applyAndReset();

		// repetition counter = 1
		// only trigger interrupt on timer underflow in center-aligned mode
		// must be set directly after starting the timer
		TIM1->RCR = 1;
		// 0b1101: "tim_oc4refc rising or tim_oc6refc falling edges generate pulses on tim_trgo2"
		// 0111: Compare - tim_oc4refc signal is used as trigger output (tim_trgo2)
		TIM1->CR2 |= (0b0111 << TIM_CR2_MMS2_Pos);

		MotorTimer::configureOutputChannel(4, MotorTimer::OutputCompareMode::Pwm, int(MaxPwm*0.95));

		MotorTimer::enableInterruptVector(MotorTimer::Interrupt::Update, true, 5);
		MotorTimer::enableInterrupt(MotorTimer::Interrupt::Update);

		MotorTimer::enableOutput();

		MotorTimer::pause();

		MotorTimer::connect<PhaseUN::Ch1n,
							PhaseVN::Ch2n,
							PhaseWN::Ch3n,
							PhaseUP::Ch1,
							PhaseVP::Ch2,
							PhaseWP::Ch3>();
		// initializeHall();
	}
};

namespace MotorBridge {
	using GateDriverEnable	= GpioA11;

	struct GateDriver
	{
		static constexpr auto SpiBaudrate = 100'000;

		using Cs	= GpioA3;
		using Sck	= GpioC13;
		using Miso	= GpioC15;
		using Mosi	= GpioC14;

		using Spi	= BitBangSpiMaster<Sck, Mosi, Miso>;

		static inline void
		initialize()
		{
			Cs::setOutput(true);
			Spi::connect<Sck::BitBang, Mosi::BitBang, Miso::BitBang>();
			Spi::initialize<SystemClock, SpiBaudrate>();
		}
	};

	inline void
	initialize()
	{
		GateDriverEnable::setOutput(Gpio::OutputType::PushPull);
		GateDriverEnable::set(false);
		GateDriver::initialize();

	}
}

namespace BackEmf {
	using BemfU		= GpioA0;
	using BemfV		= GpioA1;
	using BemfW		= GpioA2;

	// ... (TODO)
	inline void
	initialize()
	{
		// TODO
		BemfU::setInput(Gpio::InputType::Floating);
		BemfV::setInput(Gpio::InputType::Floating);
		BemfW::setInput(Gpio::InputType::Floating);
	}
}

namespace MotorCurrent {
	using SenseW	= GpioA4;
	using SenseV	= GpioB0;
	using SenseU	= GpioB1;
	using SenseIn	= GpioB12;
	using AdcW = Adc2;
	using AdcV = Adc1;
	using AdcU = Adc3;
	using AdcIn = Adc4;

	// using CompU = Comp1; // Comp1_INM
	// using CompV = Comp2; // Comp4_INP
	// using CompW = Comp1; // Comp...

	static int16_t offset_U;
	static int16_t offset_V;
	static int16_t offset_W;

	inline void setCurrentLimit(uint16_t limit) {
		// Limit to 12 bit
		DAC3->DHR12R1 = limit >> 4;
		DAC3->DHR12R2 = limit >> 4;
	}

	inline void calibrateOffset(){
		modm::delay_ms(50);
		// make sure, that zero current is centered
		constexpr uint16_t current_samples = 100;
		modm::filter::MovingAverage<int32_t, current_samples> average_U;
		modm::filter::MovingAverage<int32_t, current_samples> average_V;
		modm::filter::MovingAverage<int32_t, current_samples> average_W;
		for (uint16_t i = 0; i<=current_samples; i++){
			int16_t current_U = Board::MotorCurrent::AdcU::getValue();
			int16_t current_V = Board::MotorCurrent::AdcV::getValue();
			int16_t current_W = Board::MotorCurrent::AdcW::getValue();
			average_U.update(current_U-0x7ff);
			average_V.update(current_V-0x7ff);
			average_W.update(current_W-0x7ff);
			modm::delay_us(200);
		}
		offset_U = average_U.getValue();
		offset_V = average_V.getValue();
		offset_W = average_W.getValue();
	}


	inline void
	initialize()
	{
		SenseU::setAnalogInput();
		SenseV::setAnalogInput();
		SenseW::setAnalogInput();
		SenseIn::setAnalogInput();

		// Set VREFBUF output to 2.9 V
		// VREFBUF->CSR &= ~(VREFBUF_CSR_HIZ | VREFBUF_CSR_VRS_0);
		// VREFBUF->CSR |= (VREFBUF_CSR_ENVR | VREFBUF_CSR_VRS_1);

		// modm::delay_ms(20);

		// // Initialize comparator
		// CompU::initialize(CompU::InvertingInput::Dac3Ch1, CompU::NonInvertingInput::GpioA0);
		// CompV::initialize(CompV::InvertingInput::Dac3Ch1, CompV::NonInvertingInput::GpioA1);
		// CompW::initialize(CompW::InvertingInput::Dac3Ch2, CompW::NonInvertingInput::GpioA3);
		// // Connect comparators INPs to Gpios
		// // Connect DAC3_OUT1 to COMP1_INM,
		// // DAC3_OUT2 to COMP2_INM,
		// // and DAC3_OUT1 to COMP3_INM

		// CompU::setHysteresis(hysteresis);
		// CompV::setHysteresis(hysteresis);
		// CompW::setHysteresis(hysteresis);

		// // Connect COMP{1,2,3}_OUT to TIM1_BKIN
		// TIM1->AF1 |= (TIM1_AF1_BKCMP3E | TIM1_AF1_BKCMP2E | TIM1_AF1_BKCMP1E);

		// // Initialize STM32 internal DAC3
		// Rcc::enable<Peripheral::Dac3>();
		// //RCC->AHB2ENR1 |= RCC_AHB2ENR_DAC3EN;
		// DAC3->MCR = DAC_MCR_MODE1_0 | DAC_MCR_MODE2_0;
		// DAC3->CR = DAC_CR_EN1 | DAC_CR_EN2;
		// setCurrentLimit(0xFFFF / 2); // 50%

		AdcU::initialize(AdcU::ClockMode::SynchronousPrescaler4,
						 AdcU::ClockSource::SystemClock,
						 AdcU::Prescaler::Disabled,
						 AdcU::CalibrationMode::SingleEndedInputsMode);

		AdcV::initialize(AdcV::ClockMode::SynchronousPrescaler4,
						 AdcV::ClockSource::SystemClock,
						 AdcV::Prescaler::Disabled,
						 AdcV::CalibrationMode::SingleEndedInputsMode);

		AdcW::initialize(AdcW::ClockMode::SynchronousPrescaler4,
						 AdcW::ClockSource::SystemClock,
						 AdcW::Prescaler::Disabled,
						 AdcW::CalibrationMode::SingleEndedInputsMode);

		AdcIn::initialize(AdcIn::ClockMode::SynchronousPrescaler4,
						 AdcIn::ClockSource::SystemClock,
						 AdcIn::Prescaler::Disabled,
						 AdcIn::CalibrationMode::SingleEndedInputsMode);

		AdcW::connect<SenseW::In17>();
		AdcV::connect<SenseV::In15>();
		AdcU::connect<SenseU::In1>();
		AdcIn::connect<SenseIn::In3>();

		// Trigger ADC1,2,3,4 from timer 1 TRGO2
		ADC1->CFGR |= ADC_CFGR_EXTEN_0 | (10 << ADC_CFGR_EXTSEL_Pos) | ADC_CFGR_OVRMOD;
		ADC2->CFGR |= ADC_CFGR_EXTEN_0 | (10 << ADC_CFGR_EXTSEL_Pos) | ADC_CFGR_OVRMOD;
		ADC3->CFGR |= ADC_CFGR_EXTEN_0 | (10 << ADC_CFGR_EXTSEL_Pos) | ADC_CFGR_OVRMOD;
		ADC4->CFGR |= ADC_CFGR_EXTEN_0 | (10 << ADC_CFGR_EXTSEL_Pos) | ADC_CFGR_OVRMOD;

		AdcU::setPinChannel<SenseU>(AdcU::SampleTime::Cycles13);
		//AdcU::setChannel(AdcU::Channel::Channel1, AdcU::SampleTime::Cycles13);
		AdcV::setPinChannel<SenseV>(AdcV::SampleTime::Cycles13);
		//AdcV::setChannel(AdcV::Channel::Channel2, AdcV::SampleTime::Cycles13);
		AdcW::setPinChannel<SenseW>(AdcW::SampleTime::Cycles13);
		AdcIn::setPinChannel<SenseIn>(AdcIn::SampleTime::Cycles13);

		AdcU::startConversion();
		AdcV::startConversion();
		AdcW::startConversion();
		AdcIn::startConversion();
	}
}

namespace Encoder {

	namespace ABI {
		using Timer			= Timer3;
		using PinA			= GpioB5;
		using PinB			= GpioB4;
		using PinIndex		= GpioB3;

		inline Timer::Value getEncoderRaw()
		{
			return Timer::getValue();
		}

		inline void
		initialize()
		{
			Timer::enable();
			Timer::setMode(Timer::Mode::UpCounter, Timer::SlaveMode::Encoder3);
			// Overflow must be 16bit because else a lot of our motor control code will break!
			Timer::setOverflow(0xffff);

			Timer::connect<PinA::Ch2, PinB::Ch1>();

			Timer::start();
		}

	}

	namespace As5047 {

		using Mosi = GpioB5;
		using Miso = GpioB4;
		using Sck  = GpioB3;
		using Cs   = GpioA15;

		using Timer = Timer8;

		inline
		void initialize(){
			Cs::setOutput(modm::Gpio::High);
			using Connector = GpioConnector<Peripheral::Spi1, Sck::Sck, Mosi::Mosi, Miso::Miso>;
			Sck::setOutput(Gpio::OutputType::PushPull);
			Mosi::setOutput(Gpio::OutputType::PushPull);
			Miso::setInput(Gpio::InputType::Floating);
			Connector::connect();
			constexpr auto baudrate = 10.6_MHz;
			constexpr auto tolerance = modm::pct(5);
			constexpr auto result = modm::Prescaler::from_power(SystemClock::Spi1, baudrate, 2, 256);
			SpiMaster1::assertBaudrateInTolerance< result.frequency, baudrate, tolerance >();

			// translate the prescaler into the bitmapping
			constexpr SpiHal1::Prescaler prescaler{result.index << SPI_CR1_BR_Pos};
			SpiHal1::initialize(prescaler, SpiHal1::MasterSelection::Master, SpiHal1::DataMode::Mode1, SpiHal1::DataOrder::MsbFirst, SpiHal1::DataSize::Bit16);

			// Timer::connect<Cs::Ch1>();
			Timer::enable();
			Timer::setMode(
				Timer::Mode::UpCounter,
				Timer::SlaveMode::Reset,
				Timer::SlaveModeTrigger::Internal0, // LOL modm doesn't expose Internal0
				Timer::MasterMode::CompareOc1Ref,   // Oc1ref triggers tim4, tim5, tim8
				false);

			// TIM8->SMCR = static_cast<uint32_t>(Timer::SlaveMode::Reset); // trigger by tim1_trgo
			Timer::setPrescaler(1);
			// Pwm frequency: 170MHz / 2048 / 2 = 41503kHz
			Timer::setOverflow(Motor::MaxPwm*2);
			Timer::applyAndReset();
			// one timer increment is 1/170MHz = 11.76 ns is one clock cycle
			Timer::configureOutputChannel(1, Timer::OutputCompareMode::Pwm, 1.6*Motor::MaxPwm);
			Timer::configureOutputChannel(2, Timer::OutputCompareMode::Pwm, 1.6*Motor::MaxPwm+350);
			Timer::configureOutputChannel(3, Timer::OutputCompareMode::Pwm, 1.6*Motor::MaxPwm+450);
			Timer::configureOutputChannel(4, Timer::OutputCompareMode::Pwm, 1.6*Motor::MaxPwm+750);

			// Timer::configureOutputChannel(1, Timer::OutputCompareMode::Pwm, static_cast<uint16_t>(MaxPwm/16));
			// Timer::enableOutput();

			Timer::enableInterruptVector(Timer::Interrupt::CaptureCompare1, true, 4);
			Timer::enableInterrupt(Timer::Interrupt::CaptureCompare1);
			Timer::enableInterrupt(Timer::Interrupt::CaptureCompare2);
			Timer::enableInterrupt(Timer::Interrupt::CaptureCompare3);
			Timer::enableInterrupt(Timer::Interrupt::CaptureCompare4);

			Timer::applyAndReset();
			Timer::pause();
		}

	}

}

// namespace Sensor {
// 	using Switch		= GpioA15;

// 	//using Adc 			= Adc??
// 	using Ntc			= GpioB1; // ADC1_IN12 or ADC3_IN1
// 	using Vsupply		= GpioB2; // ADC2_IN12

// 	inline void
// 	initialize()
// 	{
// 		// external pull-up
// 		Switch::setInput(Gpio::InputType::Floating);
// 		Ntc::setAnalogInput();
// 		Vsupply::setAnalogInput();

// 		// TODO: Adc...
// 	}
// }

namespace CanBus {
	using CanRx			= GpioB8;
	using CanTx			= GpioB9;
	using Can			= Fdcan1;

	static constexpr uint32_t CanBaudrate = 1_Mbps;

	inline void
	initialize()
	{
		Can::connect<CanRx::Rx, CanTx::Tx>(Gpio::InputType::PullUp);
		Can::initialize<SystemClock, CanBaudrate>(9);
	}
}

inline void
initializeMcu()
{
	SystemClock::enable();
	modm::platform::SysTickTimer::initialize<SystemClock>();
}

inline void
initializeAllPeripherals()
{
	Ui::initializeDebugUart();
	Motor::initialize();
	MotorCurrent::initialize();
	MotorBridge::initialize();
	Encoder::As5047::initialize();
	// Sensor::initialize();
	CanBus::initialize();
}

}

#endif
