/*
 * Copyright (c) 2013-2014, Kevin Läufer
 * Copyright (c) 2014, 2016-2018, Niklas Hauser
 * Copyright (c) 2017, Sascha Schade
 * Copyright (c) 2022-2023, Christopher Durand
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32F3_ADC3_HPP
#define MODM_STM32F3_ADC3_HPP

#include <cstdint>
#include <span>
#include <utility>
#include "../device.hpp"
#include <modm/architecture/interface/register.hpp>
#include <modm/platform/gpio/connector.hpp>
#include <modm/math/algorithm/enumerate.hpp>

namespace modm
{
namespace platform
{
/**
 * Analog/Digital-Converter module (ADC3).
 *
 * The 12-bit ADC is a successive approximation analog-to-digital
 * converter. It has up to 19 multiplexed channels allowing it measure
 * signals from 16 external and three internal sources.
 * The result of the ADC is stored in a left-aligned or right-aligned
 * 16-bit data register.
 *
 * This API is designed for the internal ADCs of STM32F30X/STM32F31X
 *
 * \author	Kevin Laeufer
 * \author	Sascha Schade (strongly-typed)
 * \ingroup	modm_platform_adc modm_platform_adc_3
 */
class Adc3
{

public:
	static constexpr uint8_t Resolution = 12;

public:
	/// Channels, which can be used with this ADC.
	enum class Channel : uint8_t	// TODO: What is the best type?
	{
		Channel1 = 1,
		Channel2 = 2,
		Channel3 = 3,
		Channel4 = 4,
		Channel5 = 5,
		Channel6 = 6,
		Channel7 = 7,
		Channel8 = 8,
		Channel9 = 9,
		Channel10 = 10,
		Channel11 = 11,
		Channel12 = 12,
		Channel14 = 14,
		Channel15 = 15,
		Channel16 = 16,

		Opamp3 = 13,
		BatDiv3 = 17,
		InternalReference = 18,
	};

	enum class ClockMode : uint32_t
	{
		DoNotChange = 0xff,// if you do not want to change the clock mode
		Asynchronous = 0,	// clocked by ADC_CK12 / ADC_CK34 / ADC123_CK
		// In synchronous mode ADC is clocked by the AHB clock (stm32f3) or
		// by HCLK (stm32l4)
		SynchronousPrescaler1 = ADC_CCR_CKMODE_0,
		SynchronousPrescaler2 = ADC_CCR_CKMODE_1,
		SynchronousPrescaler4 = ADC_CCR_CKMODE_1 | ADC_CCR_CKMODE_0,
	};

	// ADCs clock source selection
	enum class ClockSource : uint32_t
	{
		NoClock = 0, // No clock selected.
		Pll = RCC_CCIPR_ADC345SEL_0, // PLL “P” clock selected as ADC clock
		SystemClock = RCC_CCIPR_ADC345SEL_1 , // System clock selected as ADCs clock
	};
	// Prescaler of the Asynchronous ADC clock
	enum class Prescaler : uint32_t
	{
		Disabled 			= 0,
		Div1 				= 0,
		Div2 				= ADC_CCR_PRESC_0,
		Div4 				= ADC_CCR_PRESC_1,
		Div6 				= ADC_CCR_PRESC_1 | ADC_CCR_PRESC_0,
		Div8 				= ADC_CCR_PRESC_2,
		Div10 				= ADC_CCR_PRESC_2 | ADC_CCR_PRESC_0,
		Div12 				= ADC_CCR_PRESC_2 | ADC_CCR_PRESC_1,
		Div16 				= ADC_CCR_PRESC_2 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_0,
		Div32 				= ADC_CCR_PRESC_3,
		Div64 				= ADC_CCR_PRESC_3 | ADC_CCR_PRESC_0,
		Div128 				= ADC_CCR_PRESC_3 | ADC_CCR_PRESC_1,
		Div256 				= ADC_CCR_PRESC_3 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_0,
		Div256AllBits 		= ADC_CCR_PRESC_3 | ADC_CCR_PRESC_2 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_0,	// for bit clear
	};

	enum class SampleTime : uint8_t	// TODO: What is the best type?
	{
		Cycles2 	= 0b000,	//<   1.5 ADC clock cycles
		Cycles7 	= 0b001,	//<   6.5 ADC clock cycles
		Cycles13 	= 0b010,	//<  12.5 ADC clock cycles
		Cycles25	= 0b011,	//<  24.5 ADC clock cycles
		Cycles48 	= 0b100,	//<  47.5 ADC clock cycles
		Cycles93 	= 0b101,	//<  92.5 ADC clock cycles
		Cycles248 	= 0b110,	//< 247.5 ADC clock cycles
		Cycles641 	= 0b111,	//< 640.5 ADC clock cycles
	};
	struct SequenceChannel
	{
		Channel channel{};
		SampleTime sampleTime{};
	};

	enum class CalibrationMode : uint32_t
	{
		SingleEndedInputsMode = 0,
		DifferntialInputsMode = ADC_CR_ADCALDIF,
		DoNotCalibrate = 0xff,	// if you want to avoid calibration
	};

	enum class VoltageRegulatorState : uint32_t
	{
		Enabled 		= ADC_CR_ADVREGEN,
	};

	enum class DmaMode : uint32_t
	{
		Disabled = 0,
		OneShot = ADC_CFGR_DMAEN,
		Circular = ADC_CFGR_DMACFG | ADC_CFGR_DMAEN,
		Mask = Circular
	};

	enum class Interrupt : uint32_t
	{
		Ready 								= ADC_IER_ADRDYIE,
		EndOfSampling 						= ADC_IER_EOSMPIE,
		EndOfRegularConversion 				= ADC_IER_EOCIE,
		EndOfRegularSequenceOfConversions 	= ADC_IER_EOSIE,
		Overrun 							= ADC_IER_OVRIE,
		EndOfInjectedConversion 			= ADC_IER_JEOCIE,
		EndOfInjectedSequenceOfConversions 	= ADC_IER_JEOSIE,
		AnalogWatchdog1 					= ADC_IER_AWD1IE,
		AnalogWatchdog2 					= ADC_IER_AWD2IE,
		AnalogWatchdog3 					= ADC_IER_AWD3IE,
		InjectedContextQueueOverflow 		= ADC_IER_JQOVFIE,
	};
	MODM_FLAGS32(Interrupt);

	enum class InterruptFlag : uint32_t
	{
		Ready 								= ADC_ISR_ADRDY,
		EndOfSampling 						= ADC_ISR_EOSMP,
		EndOfRegularConversion 				= ADC_ISR_EOC,
		EndOfRegularSequenceOfConversions 	= ADC_ISR_EOS,
		Overrun 							= ADC_ISR_OVR,
		EndOfInjectedConversion 			= ADC_ISR_JEOC,
		EndOfInjectedSequenceOfConversions 	= ADC_ISR_JEOS,
		AnalogWatchdog1 					= ADC_ISR_AWD1,
		AnalogWatchdog2 					= ADC_ISR_AWD2,
		AnalogWatchdog3 					= ADC_ISR_AWD3,
		InjectedContextQueueOverflow 		= ADC_ISR_JQOVF,
		All									= ADC_ISR_ADRDY | ADC_ISR_EOSMP | ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR | ADC_ISR_JEOC | ADC_ISR_JEOS | ADC_ISR_AWD1 | ADC_ISR_AWD2 | ADC_ISR_AWD3 | ADC_ISR_JQOVF,
	};
	MODM_FLAGS32(InterruptFlag);

public:
	template< class... Signals >
	static void
	connect()
	{
		using Connector = GpioConnector<Peripheral::Adc3, Signals...>;
		Connector::connect();
	}

	/**
	 * Initialize and enable the A/D converter.
	 *
	 * Enables the ADC clock and switches on the ADC. The ADC clock
	 * prescaler will be set as well.
	 *
	 * The ADC can be clocked
	 *
	 * @param clk
	 * 		Clock Mode for ADC1/ADC2 or ADC3/ADC4.
	 * 		Set to ClockMode::DoNotChange or leave blank if you
	 * 		want to leave this setting untouched.
	 *
	 * @param pre
	 * 		The prescaler for the asynchronous ADC clock.
	 * 		This parameter is only taken into account
	 * 		if clk == ClockMode::Asynchronous.
	 */
	static inline void
	initialize(	const ClockMode clk = ClockMode::DoNotChange,
				const ClockSource clk_src = ClockSource::SystemClock,
				const Prescaler pre = Prescaler::Disabled,
				const CalibrationMode cal = CalibrationMode::DoNotCalibrate,
				const bool blocking = true);

	static inline void
	disable(const bool blocking = true);

	/**
	 * Select the frequency of the clock to the ADC. The clock is common
	 * for ADC1/ADC2 and ADC3/ADC4.
	 *
	 * @pre The PLL must be running.
	 *
	 * @param prescaler
	 * 		The prescaler specifies by which factor the system clock
	 * 		will be divided.
	 */
	static inline void
	setPrescaler(const Prescaler pre);

	/**
	 * Returns true if the ADRDY bit of the ISR is set
	 **/
	static inline bool
	isReady();

	static inline void
	calibrate(const CalibrationMode mode, const bool blocking = true);

	/**
	 * Change the presentation of the ADC conversion result.
	 *
	 * @param enable
	 * 		Set to \c true to left adjust the result.
	 *		Otherwise, the result is right adjusted.
	 *
	 * @pre The ADC clock must be started and the ADC switched on with
	 * 		initialize()
	 */
	static inline void
	setLeftAdjustResult(const bool enable);
	/**
	 * Analog channel selection.
	 *
	 * This not for scan mode. The number of channels will be set to 1,
	 * the channel selected and the corresponding pin will be set to
	 * analog input.
	 * If the the channel is modified during a conversion, the current
	 * conversion is reset and a new start pulse is sent to the ADC to
	 * convert the new chosen channnel / group of channels.
	 *
	 *
	 * @param channel		The channel which shall be read.
	 * @param sampleTime	The sample time to sample the input voltage.
	 *
	 * @pre The ADC clock must be started and the ADC switched on with
	 * 		initialize()
	 */
	static inline bool
	setChannel(const Channel channel,
			const SampleTime sampleTime=static_cast<SampleTime>(0b000));

	static inline bool
	setChannelSequence(std::span<const SequenceChannel> sequence);

	/// Setting the channel for a Pin
	template< class Gpio >
	static inline bool
	setPinChannel(SampleTime sampleTime = static_cast<SampleTime>(0b000))
	{
		return setChannel(getPinChannel<Gpio>(), sampleTime);
	}
	/// Get the channel for a Pin
	template< class Gpio >
	static inline constexpr Channel
	getPinChannel()
	{
		constexpr int8_t channel{detail::AdcChannel<typename Gpio::Data, Peripheral::Adc3>};
		static_assert(channel >= 0, "Adc3 does not have a channel for this pin!");
		return Channel(channel);
	}

	/**
	 * Enables free running mode
	 *
	 * The ADC will continously start conversions and provide the most
	 * recent result in the ADC register.
	 *
	 * @pre The ADC clock must be started and the ADC switched on with
	 * 		initialize()
	 */
	static inline void
	setFreeRunningMode(const bool enable);

	/**
	 * Start a new conversion or continuous conversions.
	 *
	 * @pre A ADC channel must be selected with setChannel().
	 *
	 * @post The result can be fetched with getValue()
	 *
	 * TODO: is there any limitation to when is can be called??
	 */
	static inline void
	startConversion();

	static inline void
	stopConversion();

	/**
	 * @return If the conversion is finished.
	 * @pre A conversion should have been started with startConversion()
	 */
	static inline bool
	isConversionFinished();

	static inline bool
	isConversionSequenceFinished();

	/**
	 * Start a new injected conversion sequence.
	 *
	 * @pre Channels must be selected with setInjectedConversionChannel().
	 */
	static inline void
	startInjectedConversionSequence();

	/**
	 * @arg index Index of injected conversion in sequence (0..3)
	 * @return true if configuration is successful, false if arguments are invalid
	 */
	static inline bool
	setInjectedConversionChannel(uint8_t index, Channel channel, SampleTime sampleTime);

	/**
	 * @arg index Index of injected conversion in sequence (0..3)
	 * @return true if configuration is successful, false if arguments are invalid
	 */
	template<class Gpio>
	static inline bool
	setInjectedConversionChannel(uint8_t index, SampleTime sampleTime);

	/**
	 * @arg length Length of injected conversion sequence (1..4)
	 * @return true if configuration is successful, false if arguments are invalid
	 */
	static inline bool
	setInjectedConversionSequenceLength(uint8_t length);

	/**
	 * @return If the injected conversion sequence is finished.
	 * @pre An injected conversion should have been started with startInjectedConversionSequence()
	 */
	static inline bool
	isInjectedConversionFinished();

	/**
	 * @return The most recent result of the ADC conversion.
	 * @pre A conversion should have been started with startConversion()
	 *
	 * To have a blocking GET you might do it this way:
	 * @code
		while(!isConversionFinished())
		{
			// Waiting for conversion
		}
		@endcode
	 */
	static inline auto
	getValue()
	{
		return ADC3->DR;
	}

	/**
	 * Get result of injected conversion.
	 * @pre The injected conversion sequence is completed.
	 */
	static inline auto
	getInjectedConversionValue(uint8_t index)
	{
		switch (index) {
		case 0:
			return ADC3->JDR1;
		case 1:
			return ADC3->JDR2;
		case 2:
			return ADC3->JDR3;
		case 3:
			[[fallthrough]];
		default:
			return ADC3->JDR4;
		}
	}

	static inline void
	enableInterruptVector(const uint32_t priority, const bool enable = true);

	static inline void
	enableInterrupt(const Interrupt_t interrupt);

	static inline void
	disableInterrupt(const Interrupt_t interrupt);

	static inline InterruptFlag_t
	getInterruptFlags();

	static inline void
	acknowledgeInterruptFlags(const InterruptFlag_t flags);

	/// @return ADC data register pointer, for DMA use only.
	static inline volatile uint32_t*
	dataRegister()
	{
		return &(ADC3->DR);
	}

	static inline void
	setDmaMode(DmaMode mode);

private:
	static inline bool
	configureChannel(Channel channel, SampleTime sampleTime);
};

}

}

#include "adc_3_impl.hpp"

#endif	// MODM_STM32F3_ADC3_HPP
