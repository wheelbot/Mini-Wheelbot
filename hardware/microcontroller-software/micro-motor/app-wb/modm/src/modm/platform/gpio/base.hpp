/*
 * Copyright (c) 2016-2018, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_GPIO_BASE_HPP
#define MODM_STM32_GPIO_BASE_HPP

#include "../device.hpp"
#include <modm/architecture/interface/gpio.hpp>
#include <modm/math/utils/bit_operation.hpp>
#include <modm/platform/core/peripherals.hpp>

namespace modm::platform
{

/// @ingroup modm_platform_gpio
struct Gpio
{
	enum class
	InputType
	{
		Floating = 0x0,	///< floating on input
		PullUp = 0x1,	///< pull-up on input
		PullDown = 0x2,	///< pull-down on input
	};

	enum class
	OutputType
	{
		PushPull = 0x0,		///< push-pull on output
		OpenDrain = 0x1,	///< open-drain on output
	};

	enum class
	OutputSpeed
	{
		Low      = 0,
		Medium   = 0x1,
		High     = 0x2,
		VeryHigh = 0x3,		///< 30 pF (80 MHz Output max speed on 15 pF)
		MHz2     = Low,
		MHz25    = Medium,
		MHz50    = High,
		MHz100   = VeryHigh,
	};

	/// The Port a Gpio Pin is connected to.
	enum class
	Port
	{
		A = 0,
		B = 1,
		C = 2,
		F = 5,
		G = 6,
	};

	/// @cond
	enum class
	Signal
	{
		BitBang,
		Bk1io0,
		Bk1io1,
		Bk1io2,
		Bk1io3,
		Bk1ncs,
		Bk2io1,
		Bkin,
		Bkin2,
		Cc1,
		Cc2,
		Ch1,
		Ch1n,
		Ch2,
		Ch2n,
		Ch3,
		Ch3n,
		Ch4,
		Ch4n,
		Cha1,
		Cha2,
		Chb1,
		Chb2,
		Chc1,
		Chc2,
		Chd1,
		Chd2,
		Ck,
		Ck1,
		Ck2,
		Ckin,
		Clk,
		Crssync,
		Cts,
		D1,
		D2,
		Dbcc1,
		Dbcc2,
		De,
		Dm,
		Dp,
		Eev3,
		Eev4,
		Eev5,
		Eev6,
		Eev7,
		Eev8,
		Eev9,
		Etr,
		Flt1,
		Flt2,
		Flt3,
		Flt4,
		Flt5,
		Frstx1,
		Frstx2,
		Fsa,
		Fsb,
		In1,
		In10,
		In11,
		In12,
		In13,
		In14,
		In15,
		In17,
		In2,
		In3,
		In4,
		In5,
		Inm,
		Inp,
		Jtck,
		Jtdi,
		Jtdo,
		Jtms,
		Jtrst,
		Lsco,
		Mck,
		Mclka,
		Mclkb,
		Mco,
		Miso,
		Mosi,
		Nss,
		Osc32in,
		Osc32out,
		Oscin,
		Oscout,
		Out,
		Out1,
		Out2,
		Pvdin,
		Refin,
		Rts,
		Rx,
		Scin,
		Sck,
		Scka,
		Sckb,
		Scl,
		Scout,
		Sd,
		Sda,
		Sdb,
		Smba,
		Swclk,
		Swdio,
		Swo,
		Tamp1,
		Tamp2,
		Ts,
		Tx,
		Vinm,
		Vinm0,
		Vinm1,
		Vinmsec,
		Vinp,
		Vinpsec,
		Vout,
		Wkup1,
		Wkup2,
		Wkup4,
		Ws,
	};
	/// @endcond

protected:
	/// @cond
	/// I/O Direction Mode values for this specific pin.
	enum class
	Mode
	{
		Input  = 0x0,
		Output = 0x1,
		AlternateFunction = 0x2,
		Analog = 0x3,
		Mask   = 0x3,
	};

	static constexpr uint32_t
	i(Mode mode) { return uint32_t(mode); }
	// Enum Class To Integer helper functions.
	static constexpr uint32_t
	i(InputType pull) { return uint32_t(pull); }
	static constexpr uint32_t
	i(OutputType out) { return uint32_t(out); }
	static constexpr uint32_t
	i(OutputSpeed speed) { return uint32_t(speed); }
	/// @endcond
};

} // namespace modm::platform

#endif // MODM_STM32_GPIO_BASE_HPP
