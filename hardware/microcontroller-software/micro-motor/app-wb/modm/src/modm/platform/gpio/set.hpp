/*
 * Copyright (c) 2018, Niklas Hauser
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

namespace modm::platform
{

/// @ingroup modm_platform_gpio
template< class... Gpios >
class GpioSet : public Gpio
{
protected:
	static constexpr uint16_t inverteds[5] = {
		(((Gpios::port == Port::A and Gpios::isInverted) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::B and Gpios::isInverted) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::C and Gpios::isInverted) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::F and Gpios::isInverted) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::G and Gpios::isInverted) ? Gpios::mask : 0) | ...),
	};
	static constexpr uint16_t inverted(uint8_t id) { return inverteds[id]; }

	static constexpr uint16_t masks[5] = {
		(((Gpios::port == Port::A) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::B) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::C) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::F) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::G) ? Gpios::mask : 0) | ...),
	};
	static constexpr uint16_t mask(uint8_t id) { return masks[id]; }
	static constexpr uint32_t mask2(uint8_t id, uint8_t value = 0b11) {
		uint32_t r{0};
		for (int ii=0; ii<16; ii++)
			if (masks[id] & (1 << ii)) r |= (uint32_t(value) << (ii * 2));
		return r;
	}
	static constexpr uint8_t numberOfPorts() {
		uint8_t r{0};
		for (const auto &m: masks) r += (m) ? 1 : 0;
		return r;
	}

public:
	static constexpr uint8_t width = sizeof...(Gpios);
	static constexpr uint8_t number_of_ports = numberOfPorts();
public:
	static void setOutput()
	{
		if constexpr (mask(0)) GPIOA->MODER = (GPIOA->MODER & ~mask2(0)) | mask2(0, i(Mode::Output));
		if constexpr (mask(1)) GPIOB->MODER = (GPIOB->MODER & ~mask2(1)) | mask2(1, i(Mode::Output));
		if constexpr (mask(2)) GPIOC->MODER = (GPIOC->MODER & ~mask2(2)) | mask2(2, i(Mode::Output));
		if constexpr (mask(3)) GPIOF->MODER = (GPIOF->MODER & ~mask2(3)) | mask2(3, i(Mode::Output));
		if constexpr (mask(4)) GPIOG->MODER = (GPIOG->MODER & ~mask2(4)) | mask2(4, i(Mode::Output));
	}

	static void setOutput(bool status)
	{
		set(status);
		setOutput();
	}

	static void setOutput(OutputType type, OutputSpeed speed = OutputSpeed::MHz50)
	{
		configure(type, speed);
		setOutput();
	}

	static void configure(OutputType type, OutputSpeed speed = OutputSpeed::MHz50)
	{
		if constexpr (mask(0)) {
			GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~mask2(0)) | (i(speed) * mask2(0, 0b01));
			GPIOA->OTYPER  = (GPIOA->OTYPER  & ~mask(0))  | (i(type) ? mask(0) : 0);
		}
		if constexpr (mask(1)) {
			GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~mask2(1)) | (i(speed) * mask2(1, 0b01));
			GPIOB->OTYPER  = (GPIOB->OTYPER  & ~mask(1))  | (i(type) ? mask(1) : 0);
		}
		if constexpr (mask(2)) {
			GPIOC->OSPEEDR = (GPIOC->OSPEEDR & ~mask2(2)) | (i(speed) * mask2(2, 0b01));
			GPIOC->OTYPER  = (GPIOC->OTYPER  & ~mask(2))  | (i(type) ? mask(2) : 0);
		}
		if constexpr (mask(3)) {
			GPIOF->OSPEEDR = (GPIOF->OSPEEDR & ~mask2(3)) | (i(speed) * mask2(3, 0b01));
			GPIOF->OTYPER  = (GPIOF->OTYPER  & ~mask(3))  | (i(type) ? mask(3) : 0);
		}
		if constexpr (mask(4)) {
			GPIOG->OSPEEDR = (GPIOG->OSPEEDR & ~mask2(4)) | (i(speed) * mask2(4, 0b01));
			GPIOG->OTYPER  = (GPIOG->OTYPER  & ~mask(4))  | (i(type) ? mask(4) : 0);
		}
	}

	static void setInput()
	{
		if constexpr (mask(0)) {
			GPIOA->MODER &= ~mask2(0);
			GPIOA->OTYPER &= ~mask(0);
			GPIOA->OSPEEDR &= ~mask2(0);
		}
		if constexpr (mask(1)) {
			GPIOB->MODER &= ~mask2(1);
			GPIOB->OTYPER &= ~mask(1);
			GPIOB->OSPEEDR &= ~mask2(1);
		}
		if constexpr (mask(2)) {
			GPIOC->MODER &= ~mask2(2);
			GPIOC->OTYPER &= ~mask(2);
			GPIOC->OSPEEDR &= ~mask2(2);
		}
		if constexpr (mask(3)) {
			GPIOF->MODER &= ~mask2(3);
			GPIOF->OTYPER &= ~mask(3);
			GPIOF->OSPEEDR &= ~mask2(3);
		}
		if constexpr (mask(4)) {
			GPIOG->MODER &= ~mask2(4);
			GPIOG->OTYPER &= ~mask(4);
			GPIOG->OSPEEDR &= ~mask2(4);
		}
	}

	static void setInput(InputType type)
	{
		configure(type);
		setInput();
	}

	static void setAnalogInput()
	{
		if constexpr (mask(0)) GPIOA->MODER |= mask2(0, i(Mode::Analog));
		if constexpr (mask(1)) GPIOB->MODER |= mask2(1, i(Mode::Analog));
		if constexpr (mask(2)) GPIOC->MODER |= mask2(2, i(Mode::Analog));
		if constexpr (mask(3)) GPIOF->MODER |= mask2(3, i(Mode::Analog));
		if constexpr (mask(4)) GPIOG->MODER |= mask2(4, i(Mode::Analog));
	}

	static void configure(InputType type)
	{
		if constexpr (mask(0)) {
			GPIOA->PUPDR = (GPIOA->PUPDR & ~mask2(0)) | (i(type) * mask2(0, 0b01));
		}
		if constexpr (mask(1)) {
			GPIOB->PUPDR = (GPIOB->PUPDR & ~mask2(1)) | (i(type) * mask2(1, 0b01));
		}
		if constexpr (mask(2)) {
			GPIOC->PUPDR = (GPIOC->PUPDR & ~mask2(2)) | (i(type) * mask2(2, 0b01));
		}
		if constexpr (mask(3)) {
			GPIOF->PUPDR = (GPIOF->PUPDR & ~mask2(3)) | (i(type) * mask2(3, 0b01));
		}
		if constexpr (mask(4)) {
			GPIOG->PUPDR = (GPIOG->PUPDR & ~mask2(4)) | (i(type) * mask2(4, 0b01));
		}
	}

	static void set()
	{
		if constexpr (mask(0)) GPIOA->BSRR = (inverted(0) << 16) | (mask(0) & ~inverted(0));
		if constexpr (mask(1)) GPIOB->BSRR = (inverted(1) << 16) | (mask(1) & ~inverted(1));
		if constexpr (mask(2)) GPIOC->BSRR = (inverted(2) << 16) | (mask(2) & ~inverted(2));
		if constexpr (mask(3)) GPIOF->BSRR = (inverted(3) << 16) | (mask(3) & ~inverted(3));
		if constexpr (mask(4)) GPIOG->BSRR = (inverted(4) << 16) | (mask(4) & ~inverted(4));
	}

	static void set(bool status)
	{
		if (status) set();
		else        reset();
	}

	static void reset()
	{
		if constexpr (mask(0)) GPIOA->BSRR = ((uint32_t(mask(0)) & ~inverted(0)) << 16) | inverted(0);
		if constexpr (mask(1)) GPIOB->BSRR = ((uint32_t(mask(1)) & ~inverted(1)) << 16) | inverted(1);
		if constexpr (mask(2)) GPIOC->BSRR = ((uint32_t(mask(2)) & ~inverted(2)) << 16) | inverted(2);
		if constexpr (mask(3)) GPIOF->BSRR = ((uint32_t(mask(3)) & ~inverted(3)) << 16) | inverted(3);
		if constexpr (mask(4)) GPIOG->BSRR = ((uint32_t(mask(4)) & ~inverted(4)) << 16) | inverted(4);
	}

	static void toggle()
	{
		if constexpr (mask(0)) {
			const uint32_t are_set = (GPIOA->ODR & mask(0));
			const uint32_t are_reset = mask(0) ^ are_set;
			GPIOA->BSRR = (are_set << 16) | are_reset;
		}
		if constexpr (mask(1)) {
			const uint32_t are_set = (GPIOB->ODR & mask(1));
			const uint32_t are_reset = mask(1) ^ are_set;
			GPIOB->BSRR = (are_set << 16) | are_reset;
		}
		if constexpr (mask(2)) {
			const uint32_t are_set = (GPIOC->ODR & mask(2));
			const uint32_t are_reset = mask(2) ^ are_set;
			GPIOC->BSRR = (are_set << 16) | are_reset;
		}
		if constexpr (mask(3)) {
			const uint32_t are_set = (GPIOF->ODR & mask(3));
			const uint32_t are_reset = mask(3) ^ are_set;
			GPIOF->BSRR = (are_set << 16) | are_reset;
		}
		if constexpr (mask(4)) {
			const uint32_t are_set = (GPIOG->ODR & mask(4));
			const uint32_t are_reset = mask(4) ^ are_set;
			GPIOG->BSRR = (are_set << 16) | are_reset;
		}
	}

	static void lock()
	{
		if constexpr (mask(0)) {
			GPIOA->LCKR = 0x10000 | mask(0);
			GPIOA->LCKR = 0x00000 | mask(0);
			GPIOA->LCKR = 0x10000 | mask(0);
			(void) GPIOA->LCKR;
		}
		if constexpr (mask(1)) {
			GPIOB->LCKR = 0x10000 | mask(1);
			GPIOB->LCKR = 0x00000 | mask(1);
			GPIOB->LCKR = 0x10000 | mask(1);
			(void) GPIOB->LCKR;
		}
		if constexpr (mask(2)) {
			GPIOC->LCKR = 0x10000 | mask(2);
			GPIOC->LCKR = 0x00000 | mask(2);
			GPIOC->LCKR = 0x10000 | mask(2);
			(void) GPIOC->LCKR;
		}
		if constexpr (mask(3)) {
			GPIOF->LCKR = 0x10000 | mask(3);
			GPIOF->LCKR = 0x00000 | mask(3);
			GPIOF->LCKR = 0x10000 | mask(3);
			(void) GPIOF->LCKR;
		}
		if constexpr (mask(4)) {
			GPIOG->LCKR = 0x10000 | mask(4);
			GPIOG->LCKR = 0x00000 | mask(4);
			GPIOG->LCKR = 0x10000 | mask(4);
			(void) GPIOG->LCKR;
		}
	}

	static void disconnect()
	{
		(Gpios::disconnect(), ...);
	}
};

} // namespace modm::platform
