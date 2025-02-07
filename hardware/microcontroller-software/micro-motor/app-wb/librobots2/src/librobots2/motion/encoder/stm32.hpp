/*
 * Copyright (c) 2014, Kevin Läufer
 * Copyright (c) 2014, strongly-typed
 * Copyright (c) 2016, Sascha Schade
 * Copyright (c) 2018, Niklas Hauser
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

// coding: utf-8
/* Copyright (c) 2014, Roboterclub Aachen e.V.
 * All Rights Reserved.
 *
 * The file is part of the modm library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */
// ----------------------------------------------------------------------------


#ifndef LIBROBOTS2_MOTION_ENCODER_STM32_HPP
#define LIBROBOTS2_MOTION_ENCODER_STM32_HPP


/* Example Hardware Configuration
 * ==============================
 *  struct EncoderAConfiguration {
 *  	typedef GpioChannelB4 ChannelA;
 *  	typedef GpioChannelB5 ChannelB;
 *		typedef Timer modm::platform::Timer3;
 *		static inline void connect() {
 *			ChannelA::configure(Gpio::InputType::Floating);
 *			ChannelB::configure(Gpio::InputType::Floating);
 *			Timer::connect<ChannelA::Ch1, ChannelB::Ch2>();
 *		}
 *  }
*/
namespace librobots2::motion::encoder::stm32
{

/**
 * Use a Timer of STM32 in slave mode to read a quadrature encoder.
 *
 * Configuration parameter Timer must be one of the Advanced Timers.
 * Template parameter ChannelA and ChannelB must be channel 1 and 2 pins
 * of the corresponding timer connected to signal A and B, respectively,
 * of the quadrature encoder.
 *
 * run() must be called on a regular basis and the getEncoderRaw() returns
 * the current counter value.
 *
 * This encoder can then be used in encoder differentiation.
 *
 * Example:
 *	struct EncoderConfiguration {
 *		typedef Timer1          Timer;
 *		typedef GpioInputA8     ChannelA;
 *		static constexpr Gpio::InputType ChannelAInputType =
 *			Gpio::InputType::Floating;	// there are external pullups to 5V
 *		typedef GpioInputA9     ChannelB;
 *		static constexpr Gpio::InputType ChannelBInputType =
 *			Gpio::InputType::Floating;	// there are external pullups to 5V
 *	};
 *
 * typedef Encoder< EncoderConfiguration > Stm32EncoderLeft;
 *
 * Run once:
 *   initialize();
 *
 * Run on regular basis (e.g. 1 ms in interrupt handler)
 * 	run()
 *
 * Then use
 *  getCounterRaw()
 * to get the latest counter value
 */

template<typename Configuration>
class Encoder
{
private:
	/// gpio pin connected to channel A
	typedef typename Configuration::ChannelA ChannelA;
	/// gpio pin connected to channel B
	typedef typename Configuration::ChannelB ChannelB;
	/// input configuration of channel A
	static constexpr modm::platform::Gpio::InputType ChannelAInputType
		= Configuration::ChannelAInputType;
	/// input configuration of channel B
	static constexpr modm::platform::Gpio::InputType ChannelBInputType
		= Configuration::ChannelAInputType;
	/// advanced or general purpose timer that will be conncted to inputs A and B
	typedef typename Configuration::Timer Timer;

public:
	typedef uint16_t CounterT;

	/// Intializes the timer and pins in encoder mode.
	static inline void
	initialize()
	{
		Timer::enable();
		Timer::setMode(Timer::Mode::UpCounter, Timer::SlaveMode::Encoder3);
		// Overflow must be 16bit because else a lot of our motor control code will break!
		Timer::setOverflow(0xffff);
		Configuration::connect();
		Timer::start();
	};

	// Sample the counter value to member variable
	static inline void
	run()
	{
		counterRaw = Timer::getValue();
	}

	// Get the raw counter value from last sample point
	static inline CounterT
	getCounterRaw()
	{
		return counterRaw;
	}

private:
	static
	CounterT counterRaw;
};

template<typename Configuration>
typename Encoder<Configuration>::CounterT
Encoder<Configuration>::counterRaw;

}	// namespace librobots2::motion::encoder::stm32

#endif // LIBROBOTS2_MOTION_ENCODER_STM32_HPP
