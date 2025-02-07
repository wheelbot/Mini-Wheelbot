/*
 * Copyright (c) 2020, Raphael Lehmann
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef PDB_HARDWARE_V1_HPP
#define PDB_HARDWARE_V1_HPP

#include <modm/platform.hpp>
#include <modm/architecture/interface/clock.hpp>
#include <modm/platform/comp/comp_2.hpp>
#include <modm/platform/dac/dac_1.hpp>
#include <modm/platform/dac/dac_3.hpp>
#include <modm/debug/logger.hpp>
#include <modm/platform/uart/uart_1.hpp>
#include <modm/platform/timer/timer_1.hpp>
#include <modm/platform/timer/timer_2.hpp>
#include <modm/platform/timer/timer_8.hpp>
#include <modm/platform/spi/spi_master_1.hpp>
#include <modm/platform/spi/spi_master_3.hpp>
#include <modm/platform/dma/dma.hpp>
#include <modm/platform/exti/exti.hpp>
#include <modm/platform/id/id.hpp>
#include <modm/driver/radio/nrf24.hpp>
#include <modm/debug/logger.hpp>

#define MODM_BOARD_HAS_LOGGER

using namespace modm::platform;
using namespace std::chrono_literals;

namespace Board
{
using namespace modm::literals;

/// STM32G431RB running at 170MHz generated from the internal 16MHz crystal
struct SystemClock
{
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
	static constexpr uint32_t Dac       = Ahb2;
	static constexpr uint32_t Dac1      = Dac;
	static constexpr uint32_t Dac3      = Dac;
	static constexpr uint32_t Rng       = Ahb2;

	static constexpr uint32_t Can       = Apb1;
	static constexpr uint32_t Fdcan1    = Can;
	static constexpr uint32_t I2c       = Apb1;
	static constexpr uint32_t I2c1      = I2c;
	static constexpr uint32_t I2c2      = I2c;
	static constexpr uint32_t I2c3      = I2c;
	static constexpr uint32_t Lpuart    = Apb1;
	static constexpr uint32_t Rtc       = Apb1;
	static constexpr uint32_t Spi2      = Apb1;
	static constexpr uint32_t Spi3      = Apb1;
	static constexpr uint32_t Usart2    = Apb1;
	static constexpr uint32_t Usart3    = Apb1;
	static constexpr uint32_t Usb       = Apb1;
	static constexpr uint32_t Apb1Timer = Apb1 * 1;
	static constexpr uint32_t Timer2    = Apb1Timer;
	static constexpr uint32_t Timer3    = Apb1Timer;
	static constexpr uint32_t Timer4    = Apb1Timer;
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

	static bool inline
	enable()
	{
		Rcc::enableInternalClock();	// 16MHz
		Rcc::PllFactors pllFactors{
			.pllM = 4,	//  16MHz / M= 4 ->   4MHz
			.pllN = 85,	//   4MHz * N=85 -> 340MHz
			.pllR = 2,	// 340MHz / R= 2 -> 170MHz = F_cpu
		};
		Rcc::enablePll(Rcc::PllSource::InternalClock, pllFactors);
		Rcc::setFlashLatency<Frequency>();
		Rcc::setVoltageScaling(Rcc::VoltageScaling::Boost); // recommended for >150 MHz
		// switch system clock to PLL output
		Rcc::enableSystemClock(Rcc::SystemClockSource::Pll);
		Rcc::setAhbPrescaler(Rcc::AhbPrescaler::Div1);
		// APB1 has max. 170MHz
		Rcc::setApb1Prescaler(Rcc::Apb1Prescaler::Div1);
		Rcc::setApb2Prescaler(Rcc::Apb2Prescaler::Div1);
		// update frequencies for busy-wait delay functions
		Rcc::updateCoreFrequency<Frequency>();

		return true;
	}
};


namespace Ui{

    // using Button = GpioInverted<GpioInputC13>;
    using LedGreen      = GpioOutputB13;
    using LedRed        = GpioOutputB14;

    using DebugUartRx   = GpioInputA10;
    using DebugUartTx   = GpioOutputA9;
    using DebugUart     = BufferedUart<UsartHal1,  UartRxBuffer<2048>, UartTxBuffer<2048>>;

    static constexpr uint32_t DebugUartBaudrate = 460800_Bd;

	using Buzzer = GpioC13;
	using BuzzerTimer = Timer1;

	inline void
	initializeDebugUart()
	{
		DebugUart::connect<DebugUartTx::Tx, DebugUartRx::Rx>();
		DebugUart::initialize<SystemClock, DebugUartBaudrate>();
	}

	inline void
	initializeLeds()
	{
		LedRed::setOutput(false);
		LedGreen::setOutput(false);
	}

	inline void
	initializeBuzzer(){
		BuzzerTimer::enable();
		BuzzerTimer::setMode(BuzzerTimer::Mode::UpCounter);
		// BuzzerTimer clock: APB2 timer clock (170MHz)
		BuzzerTimer::setPrescaler(10);
		// Prescaler: 1 -> Timer counter frequency: 170MHz
		BuzzerTimer::setOverflow(4250);
		// Pwm frequency: 170MHz / 2048 / 2 = 83kHz

		// TODO: PR #1016
		// BuzzerTimer::configureOutputChannel<Buzzer::Ch1n>(BuzzerTimer::OutputCompareMode::Pwm, 0);
		BuzzerTimer::configureOutputChannel(1,
			BuzzerTimer::OutputCompareMode::Pwm,
			BuzzerTimer::PinState::Disable,
			BuzzerTimer::OutputComparePolarity::ActiveHigh,
			BuzzerTimer::PinState::Enable,
			BuzzerTimer::OutputComparePolarity::ActiveHigh,
			BuzzerTimer::OutputComparePreload::Disable
			);
		// BuzzerTimer::applyAndReset();

		BuzzerTimer::setCompareValue(1,0);
		BuzzerTimer::applyAndReset();
		BuzzerTimer::pause();

		BuzzerTimer::enableOutput();

		BuzzerTimer::connect<Buzzer::Ch1n>();

		BuzzerTimer::start();
	}

	inline void
	buzzerOn(){
		BuzzerTimer::setCompareValue(1,2048);
	}

	inline void
	buzzerOff(){
		BuzzerTimer::setCompareValue(1,0);
	}

}

modm::IODeviceWrapper< Ui::DebugUart, modm::IOBuffer::BlockIfFull > loggerDevice;

class ClockUs
{
public:
	typedef uint32_t Type;
	using ClockUsTimer = modm::platform::Timer2;

public:
	template< class SystemClock >
	static void
	enable()
	{
		ClockUsTimer::enable();
		ClockUsTimer::setMode(ClockUsTimer::Mode::UpCounter);
		ClockUsTimer::setPrescaler(SystemClock::Timer2 / 1_MHz);
		ClockUsTimer::setOverflow((uint32_t)-1);
		ClockUsTimer::applyAndReset();

		setTime(0);
		ClockUsTimer::start();
	}

	template< typename TimestampType = modm::Timestamp >
	static TimestampType
	now()
	{ return TimestampType(ClockUsTimer::getValue()); }

	static void
	setTime(const Type time)
	{ ClockUsTimer::setValue(time); }
};

namespace Nrf{
	using Spi	= SpiMaster1;
	using Sck  	= GpioB3;
	using Miso 	= GpioB4;
	using Mosi 	= GpioB5;
	using Cs 	= GpioB6;
	using En 	= GpioB7;
	using Irq 	= GpioB9;

	using Phy     = modm::Nrf24Phy<Spi, Cs, En>;
	using Config  = modm::Nrf24Config<Phy>;
	using Data    = modm::Nrf24Data<Phy, ClockUs>;

	// using Register 	 = Phy::NrfRegister;
	// using Status 	 = Phy::Status;
	// using Pipe 		 = Phy::Pipe;
	// using Config 	 = Phy::Config;
	// using FifoStatus = Phy::FifoStatus;

	void inline
	initializeSpi()
	{
		// Board::LedD13::setOutput(modm::Gpio::Low);
		ClockUs::enable<Board::SystemClock>();

		// MODM_LOG_INFO << "Initializing SPI #1..." << modm::endl;
//
		Cs::setOutput(modm::Gpio::High);
		En::setOutput(modm::Gpio::Low);
//
		Spi::connect<Sck::Sck, Mosi::Mosi, Miso::Miso>();
		Spi::initialize<Board::SystemClock, 1.3_MHz>();
	}

	// static const uint8_t nrf_receiver_address{getUniqueId(0)};
	static constexpr uint64_t base_address{0xdeadbeef00};
	static constexpr uint8_t  receiver_address{0x20};
	static constexpr uint8_t payload_length{1};
	// static uint8_t received_data[payload_length];

	void inline
	initializeNrf(uint8_t rf_channel=1)
	{
		// En::set();
		// Phy::initialize(payload_length);
		// Phy::writeRegister(Phy::NrfRegister::RF_CH, rf_channel);
		// Phy::writeRegister(Phy::NrfRegister::RX_PW_P1, payload_length);
		// Phy::setRxAddress(Phy::Pipe::PIPE_1, nrf_receiver_address);
		// Phy::setBits(Phy::NrfRegister::CONFIG, Phy::Config::PRIM_RX);
		// Phy::setBits(Phy::NrfRegister::CONFIG, Phy::Config::PWR_UP);

		MODM_LOG_INFO.printf("Initializing NRF24 #1 with address %d on channel %d...\n", receiver_address, rf_channel);

		Data::initialize(base_address, receiver_address, 0xFF);

		Config::setChannel(rf_channel);
		Config::setAutoRetransmitCount(Config::AutoRetransmitCount::Retransmit15);
		Config::setAutoRetransmitDelay(Config::AutoRetransmitDelay::us250);
		Config::setSpeed(Config::Speed::kBps250);
		Config::setCrc(Config::Crc::Crc2Byte);

		Irq::setInput(Irq::InputType::PullUp);
		Exti::connect<Irq>(Exti::Trigger::FallingEdge, [](uint8_t)
		{
			Ui::LedGreen::toggle();
			Data::interruptHandler();
		});
	}
}


namespace LogicPower{
	using EnableMotor1 	= GpioA4;
	using EnableMotor2 	= GpioC14;
	using EnablePi 		= GpioB15;

	using FaultMotor1 	= GpioA2;
	using FaultMotor2 	= GpioC15;
	using FaultPi 		= GpioA8;

	void inline
	disableAll(){
		EnableMotor1::reset();
		EnableMotor2::reset();
		EnablePi::reset();
	}

	void inline
	enableAll(){
		EnableMotor1::set();
		EnableMotor2::set();
		EnablePi::set();
	}

	void inline
	initialize(){
		EnableMotor1::setOutput();
		EnableMotor2::setOutput();
		EnablePi::setOutput();

		FaultMotor1::setInput();
		FaultMotor2::setInput();
		FaultPi::setInput();

		disableAll();

	}
}

namespace DrivePower {

	using CurrentDrive = GpioA1;
	using VoltageSupply = GpioB0;
	using VoltageDrive = GpioA3;
	using EnableDrive = GpioA6;
	using EnableBrakingL = GpioA7;
	using EnableBrakingH = GpioA15;

	using Comp = Comp2;
	using Dac = Dac1;


	using BrakingTimer = Timer8;
	// static constexpr uint16_t MaxPwm{2047}; // 11 bit PWM
	static constexpr uint16_t MaxPwm{4095}; // 11 bit PWM

	static constexpr float v_supply_div = 2.9f / 0xfff * 11.f / 0.968f;
	static constexpr float i_drive_div =  2.9f / 0xfff / 20.f / 2.e-3;
	static constexpr float v_supply_max =  25.8f;

	static inline void
	setBrakingCompareValue(uint16_t value){
		BrakingTimer::setCompareValue(1, value);
	}

	void inline
	disableDrive(){
		EnableDrive::reset();
		// setBrakingCompareValue(0.9*MaxPwm);
	}


	void inline
	enableDrive(){
		setBrakingCompareValue(0*MaxPwm);
		EnableDrive::set();
	}


	void inline
	setBrakingVoltage(const float v_bus_max=25.2){
		constexpr float voltage_divider = 10.f/(10.f+100.f);
		const auto v_pin_max = v_bus_max * voltage_divider;
		const auto dac_value = uint16_t(v_pin_max/2.9f*4095.f);
		Dac::setOutput2(dac_value);
	}

	void inline
	initializeBraking(){

		// EnableBrakingL::setOutput(false);
		EnableBrakingH::setOutput(false);

		BrakingTimer::enable();
		BrakingTimer::setMode(BrakingTimer::Mode::CenterAligned1);

		// BrakingTimer clock: APB2 timer clock (170MHz)
		BrakingTimer::setPrescaler(1);
		// Prescaler: 1 -> Timer counter frequency: 170MHz
		BrakingTimer::setOverflow(MaxPwm);
		// Pwm frequency: 170MHz / 2048 / 2 = 83kHz

		BrakingTimer::configureOutputChannel(1,
			BrakingTimer::OutputCompareMode::ForceActive,
			BrakingTimer::PinState::Enable,
			BrakingTimer::OutputComparePolarity::ActiveLow,
			BrakingTimer::PinState::Disable,
			BrakingTimer::OutputComparePolarity::ActiveHigh,
			BrakingTimer::OutputComparePreload::Disable
			);

		setBrakingCompareValue(0);

		BrakingTimer::applyAndReset();

		// repetition counter = 1
		// only trigger interrupt on timer underflow in center-aligned mode
		// must be set directly after starting the timer
		TIM8->RCR = 1;
		// 0b1101: "tim_oc4refc rising or tim_oc6refc falling edges generate pulses on tim_trgo2"
		// 0b0101: Compare - tim_oc2refc signal is used as trigger output (tim_trgo2)
		TIM8->CR2 |= (0b0101 << TIM_CR2_MMS2_Pos);

		BrakingTimer::configureOutputChannel(2, BrakingTimer::OutputCompareMode::Pwm, int(MaxPwm*0.90));

		BrakingTimer::enableInterruptVector(BrakingTimer::Interrupt::Update, true, 5);
		BrakingTimer::enableInterrupt(BrakingTimer::Interrupt::Update);

		BrakingTimer::enableOutput();

		BrakingTimer::pause();

		// BrakingTimer::connect<EnableBrakingL::Ch1n, EnableBrakingH::Ch1>();
		BrakingTimer::connect<EnableBrakingH::Ch1>();

		BrakingTimer::start();

		BrakingTimer::configureOutputChannel(1,
			BrakingTimer::OutputCompareMode::Pwm,
			BrakingTimer::PinState::Enable,
			BrakingTimer::OutputComparePolarity::ActiveHigh,
			BrakingTimer::PinState::Disable,
			BrakingTimer::OutputComparePolarity::ActiveHigh,
			BrakingTimer::OutputComparePreload::Disable
			);
	}

	// void inline
	// initializeDacComp(){

	// 	Dac::initialize<Board::SystemClock>();
	// 	Dac::connect<GpioA5::Out2>();
	// 	Dac::setMode(Dac::Channel::Channel1, Dac::Mode::Internal);
	// 	Dac::setMode(Dac::Channel::Channel2, Dac::Mode::ExternalInternalWithBuffer);
	// 	Dac::enableChannel(Dac::Channel::Channel2);


	// 	setBrakingVoltage(9.f);

	// 	__DSB();
	// 	__DMB();
	// 	__ISB();


	// 	// Comp::connect<VoltageDrive::Inp, EnableBraking::Out>();
	// 	Comp::connect<VoltageDrive::Inp>();
	// 	Comp::initialize(
	// 		Comp::InvertingInput::Dac1Ch2,
	// 		Comp::NonInvertingInput::GpioA3
	// 		);
	// 	Comp::setPolarity(Comp::Polarity::NonInverted);
	// 	Comp::setEnabled(true);

	// 	__DSB();
	// 	__DMB();
	// 	__ISB();

	// }


	void inline
	initializeAdc(){
		CurrentDrive::setAnalogInput();
		VoltageSupply::setAnalogInput();
		VoltageDrive::setAnalogInput();

		// Set VREFBUF output to 2.9 V
		VREFBUF->CSR &= ~(VREFBUF_CSR_HIZ | VREFBUF_CSR_VRS_0);
		VREFBUF->CSR |= (VREFBUF_CSR_ENVR | VREFBUF_CSR_VRS_1);

		__ISB();
		__DSB();
		modm::delay(100ms);

		__ISB();
		__DSB();

		Adc1::initialize(Adc1::ClockMode::SynchronousPrescaler4,
						 Adc1::ClockSource::SystemClock,
						 Adc1::Prescaler::Disabled,
						 Adc1::CalibrationMode::SingleEndedInputsMode,
						 true);
		Adc2::initialize(Adc2::ClockMode::SynchronousPrescaler4,
						 Adc2::ClockSource::SystemClock,
						 Adc2::Prescaler::Disabled,
						 Adc2::CalibrationMode::SingleEndedInputsMode, true);
		// Adc1::connect<VoltageSupply::In15>();
		// Adc1::connect<GpioA0::In1>();

		Adc1::connect<VoltageSupply::In15>();
		// Adc1::connect<VoltageDrive::In4>();
		Adc2::connect<CurrentDrive::In2>();
		Adc1::setPinChannel<VoltageSupply>(Adc1::SampleTime::Cycles13);
		// Adc1::setPinChannel<VoltageDrive>(Adc1::SampleTime::Cycles248);
		Adc2::setPinChannel<CurrentDrive>(Adc2::SampleTime::Cycles13);
		// Adc2::setPinChannel<CurrentDrive>(Adc2::SampleTime::Cycles248);

		// adc_ext_trg8 TIM8_TRGO2 Internal signal from on-chip timers 01000
		ADC1->CFGR |= ADC_CFGR_EXTEN_0 | (8 << ADC_CFGR_EXTSEL_Pos) | ADC_CFGR_OVRMOD;
		ADC2->CFGR |= ADC_CFGR_EXTEN_0 | (8 << ADC_CFGR_EXTSEL_Pos) | ADC_CFGR_OVRMOD;

		Adc1::startConversion();
		Adc2::startConversion();
	}

	void inline
	initialize(){

		EnableDrive::setOutput(false);

		initializeAdc();
		initializeBraking();


	}
}

inline void
initialize()
{
	SystemClock::enable();
	SysTickTimer::initialize<SystemClock>();


	Ui::initializeDebugUart();
	Ui::initializeLeds();
	Ui::initializeBuzzer();

	LogicPower::initialize();
	DrivePower::initialize();

	// DrivePower::initializeDacComp();

	Nrf::initializeSpi();
	Nrf::initializeNrf();

}

}

#endif	// PDB_HARDWARE_V1_HPP
