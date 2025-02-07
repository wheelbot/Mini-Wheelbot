/*
 * Copyright (c) 2019, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

namespace modm::platform
{

/// @ingroup modm_platform_core
enum class
Peripheral
{
	BitBang,
	Adc1,
	Adc2,
	Adc3,
	Adc4,
	Adc5,
	Comp1,
	Comp2,
	Comp3,
	Comp4,
	Comp5,
	Comp6,
	Comp7,
	Cordic,
	Crc,
	Dac1,
	Dac2,
	Dac3,
	Dac4,
	Dma1,
	Dma2,
	Dmamux1,
	Fdcan1,
	Fdcan2,
	Fdcan3,
	Flash,
	Fmac,
	Hrtim1,
	I2c1,
	I2c2,
	I2c3,
	I2c4,
	I2s,
	I2s2,
	I2s3,
	Irtim,
	Iwdg,
	Lptim1,
	Lpuart1,
	Opamp1,
	Opamp2,
	Opamp3,
	Opamp4,
	Opamp5,
	Opamp6,
	Quadspi1,
	Rcc,
	Rng,
	Rtc,
	Sai1,
	Spi1,
	Spi2,
	Spi3,
	Sys,
	Tim1,
	Tim15,
	Tim16,
	Tim17,
	Tim2,
	Tim20,
	Tim3,
	Tim4,
	Tim5,
	Tim6,
	Tim7,
	Tim8,
	Ucpd1,
	Usart1,
	Usart2,
	Usart3,
	Usb,
	Usbpd,
	Wwdg,
	Syscfg = Sys,
};

}
