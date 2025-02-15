/*
 * Copyright (c) 2017-2021, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

#include "../device.hpp"
#include "base.hpp"
#include "set.hpp"

namespace modm::platform
{

/// @ingroup	modm_platform_gpio
template< class GpioData >
class GpioStatic : public Gpio, public GpioData
{
	template<class... Gpios>
	friend class GpioSet;
	using PinSet = GpioSet<GpioStatic<GpioData>>;
public:
	using Direction = ::modm::Gpio::Direction;
	using Type = GpioStatic<GpioData>;
	using Output = Type;
	using Input = Type;
	using IO = Type;
	using Data = GpioData;
	using GpioData::port;
	using GpioData::pin;
	static constexpr Direction direction = Direction::InOut;
	static constexpr bool isInverted = false;

protected:
	/// @cond
	// Bitmask for registers that contain a 1bit value for every pin.
	static constexpr uint16_t mask  = 0x1 << pin;
	// Bitmask for registers that contain a 2bit value for every pin.
	static constexpr uint32_t mask2 = 0x3 << (pin * 2);
	// Port Number.
	static constexpr uint8_t port_nr = uint8_t(port);
	// Alternate Function register id. 0 for pin 0-7. 1 for pin 8-15.
	static constexpr uint8_t af_id  = pin / 8;
	// Alternate Function offset.
	static constexpr uint8_t af_offset = (pin * 4) % 32;
	// Alternate Function register mask.
	static constexpr uint32_t af_mask  = 0xf << af_offset;

	static GPIO_TypeDef* gport()
	{
		if constexpr (port == Gpio::Port::A) return GPIOA;
		if constexpr (port == Gpio::Port::B) return GPIOB;
		if constexpr (port == Gpio::Port::C) return GPIOC;
		if constexpr (port == Gpio::Port::F) return GPIOF;
		if constexpr (port == Gpio::Port::G) return GPIOG;
		return nullptr;
	}
	/// @endcond

public:
	inline static void setAlternateFunction(uint8_t af) {
		gport()->AFR[af_id] = (gport()->AFR[af_id] & ~af_mask) | ((af & 0xf) << af_offset);
		gport()->MODER = (gport()->MODER & ~mask2) | (i(Mode::AlternateFunction) << (pin * 2));
	}
	/// Enable Analog Mode which is needed to use this pin as an ADC input.
	inline static void setAnalogInput() { PinSet::setAnalogInput(); }

public:
	// GpioOutput
	inline static void setOutput() { PinSet::setOutput(); }
	inline static void setOutput(bool status) { PinSet::setOutput(status); }
	inline static void setOutput(OutputType type, OutputSpeed speed = OutputSpeed::MHz50) { PinSet::setOutput(type, speed); }
	inline static void configure(OutputType type, OutputSpeed speed = OutputSpeed::MHz50) { PinSet::configure(type, speed); }

	inline static void set() { PinSet::set(); }
	inline static void set(bool status) { PinSet::set(status); }
	inline static void reset() { PinSet::reset(); }
	inline static bool isSet() { return gport()->ODR & mask; }
	inline static bool toggle()
	{
		if (isSet()) { reset(); return true; }
		else         { set();   return false; }
	}

	// GpioInput
	inline static void setInput() { PinSet::setInput(); }
	inline static void setInput(InputType type) { PinSet::setInput(type); }
	inline static void configure(InputType type) { PinSet::configure(type); }

	inline static bool read() { return gport()->IDR & mask; }

	// GpioIO
	inline static Direction getDirection()
	{
		uint32_t mode = (gport()->MODER & mask2);
		if (mode == (i(Mode::Input) << pin * 2))
			return Direction::In;
		if (mode == (i(Mode::Output) << pin * 2))
			return Direction::Out;
		return Direction::Special;
	}
	inline static void lock() { PinSet::lock(); }
	inline static void disconnect()
	{
		setInput(InputType::Floating);
		gport()->AFR[af_id] &= ~af_mask;
	}
};

} // namespace modm::platform
