/*
 * Copyright (c) 2020, Jeff McBride
 * Copyright (c) 2021, Christopher Durand
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_DAC2_HPP
#define MODM_STM32_DAC2_HPP

#include <cstdint>
#include "../device.hpp"
#include <modm/platform/gpio/connector.hpp>
#include <modm/platform/clock/rcc.hpp>
#include <modm/math/units.hpp>

namespace modm::platform
{

/**
 * Digital-to-Analog Converter (DAC2)
 *
 * Supports simple synchronous access to the 2-channel DAC found on most STM32
 * microcontrollers.
 *
 * @author Jeff McBride
 * @ingroup modm_platform_dac modm_platform_dac_2
 */
class Dac2 {
public:
	/**
	 * The DAC has one output channel, Channel1
	 */
	enum class Channel : uint8_t
	{
		Channel1 = 0,
	};

	enum class Mode
	{
		ExternalWithBuffer = 0,
		ExternalInternalWithBuffer = DAC_MCR_MODE1_0,
		ExternalWithoutBuffer = DAC_MCR_MODE1_1,
		Internal = DAC_MCR_MODE1_0 | DAC_MCR_MODE1_1
	};
	template< class... Signals >
	static void
	connect()
	{
		using Connector = GpioConnector<Peripheral::Dac2, Signals...>;
		Connector::connect();
	}

	/**
	 * Initialize the D/A converter.
	 *
	 * Enables the RCC clock output for the DAC
	 *
	 * @tparam SystemClock System clock struct
	 */
	template<typename SystemClock>
	static inline void initialize();

	/** Enable DAC output for the given channel
	 *
	 * @param chan    The channel to be enabled
	 *
	 * @pre The DAC clock must be enabled with initialize()
	 */
	static void enableChannel(Channel chan);

	/**
	 * Disable DAC output for the given channel
	 *
	 * @param chan    The channel to be disabled
	 *
	 * @pre The DAC clock must be enabled with initialize()
	 */
	static void disableChannel(Channel chan);

	/**
	 * Control the output mode
	 *
	 * @param chan  The channel to configure the mode for
	 * @param mode  The channel output mode
	 *
	 * @warning The DAC channel will be disabled when setting the mode
	 */
	static void setMode(Channel chan, Mode mode);

	/**
	 * Set the output voltage for a DAC channel
	 *
	 * @param chan    The channel to set
	 * @param value   The 12-bit DAC output value, range 0-4095.
	 *
	 * @pre The DAC clock must be enabled with initialize()
	 */
	static inline void setOutput(Channel chan, uint16_t value);

	/**
	 * Set output value for Channel1
	 *
	 * Equivalent to setOutput(modm::platform::Dac::Channel1, value)
	 *
	 * @param value   The 12-bit DAC output value, range 0-4095
	 *
	 * @pre The DAC clock must be enabled with initialize()
	 */
	static inline void setOutput1(uint16_t value);

	/// Get the channel for a Pin
	template< class Gpio >
	static inline constexpr Channel
	getPinChannel()
	{
		constexpr int8_t channel{Gpio::template DacChannel<Peripheral::Dac2>};
		static_assert(channel >= 0, "Dac does not have a channel for this pin!");
		return Channel(channel);
	}

};

template <typename SystemClock>
inline void
Dac2::initialize()
{
	Rcc::enable<Peripheral::Dac2>();
	DAC2->CR = 0;
	DAC2->MCR = 0;

	if constexpr(SystemClock::Dac2 > MHz(160)) {
		DAC2->MCR = DAC_MCR_HFSEL_1;
	} else if constexpr(SystemClock::Dac2 > MHz(80)) {
		DAC2->MCR = DAC_MCR_HFSEL_0;
	}
}
inline void
Dac2::setOutput(Channel chan, uint16_t value)
{
	switch(chan) {
		case Channel::Channel1:
			DAC2->DHR12R1 = value & DAC_DHR12R1_DACC1DHR_Msk;
			break;
	}
}

inline void
Dac2::setOutput1(uint16_t value)
{
	setOutput(Channel::Channel1, value);
}

} // namespace modm::platform

#endif // MODM_STM32_DAC2_HPP
