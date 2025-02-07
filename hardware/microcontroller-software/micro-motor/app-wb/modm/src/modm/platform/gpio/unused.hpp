/*
 * Copyright (c) 2017-2018, 2021, Niklas Hauser
 * Copyright (c) 2022, Andrey Kunitsyn
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

#include "base.hpp"
#include "data.hpp"
#include "static.hpp"

namespace modm::platform
{

/// @ingroup modm_platform_gpio
using GpioUnused = GpioStatic<detail::DataUnused>;
/**
 * Dummy implementation of an I/O pin.
 *
 * This class can be used when a pin is not required. All functions
 * are dummy functions which do nothing. `read()` will always
 * return `false`.
 *
 * For example when creating a software SPI with the modm::SoftwareSimpleSpi
 * class and the return channel (MISO - Master In Slave Out) is not needed,
 * a good way is to use this class as a parameter when defining the
 * SPI class.
 *
 * Example:
 * @code
 * #include <modm/architecture/platform.hpp>
 *
 * namespace pin
 * {
 *     typedef GpioOutputD7 Clk;
 *     typedef GpioOutputD5 Mosi;
 * }
 *
 * modm::SoftwareSpiMaster< pin::Clk, pin::Mosi, GpioUnused > Spi;
 *
 * ...
 * Spi::write(0xaa);
 * @endcode
 *
 * @author	Fabian Greif
 * @author	Niklas Hauser
 * @ingroup	modm_platform_gpio
 */
template<>
class GpioStatic<detail::DataUnused> : public Gpio, public ::modm::GpioIO
{
public:
	using Output = GpioUnused;
	using Input = GpioUnused;
	using IO = GpioUnused;
	using Type = GpioUnused;
	using Data = detail::DataUnused;
	static constexpr bool isInverted = false;
	static constexpr Port port = Port(-1);
	static constexpr uint8_t pin = uint8_t(-1);
	static constexpr uint16_t mask = 0;
protected:
	static void setAlternateFunction(uint8_t) {}
	static void setAlternateFunction() {}
	static void setAnalogInput() {}
public:
	// GpioOutput
	static void setOutput() {}
	static void setOutput(bool) {}
	static void setOutput(OutputType, OutputSpeed = OutputSpeed::MHz50) {}
	static void configure(OutputType, OutputSpeed = OutputSpeed::MHz50) {}
	static void set() {}
	static void set(bool) {}
	static void reset() {}
	static bool isSet() { return false; }
	static void toggle() {}

	// GpioInput
	static void setInput() {}
	static void setInput(InputType) {}
	static void configure(InputType) {}
	static bool read() { return false; }

	// GpioIO
	static Direction getDirection() { return Direction::In; }
	static void lock() {}
	static void disconnect() {}

public:
	struct BitBang
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::BitBang;
	};
	struct Bk1io0
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Bk1io0;
	};
	struct Bk1io1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Bk1io1;
	};
	struct Bk1io2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Bk1io2;
	};
	struct Bk1io3
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Bk1io3;
	};
	struct Bk1ncs
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Bk1ncs;
	};
	struct Bk2io1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Bk2io1;
	};
	struct Bkin
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Bkin;
	};
	struct Bkin2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Bkin2;
	};
	struct Cc1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Cc1;
	};
	struct Cc2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Cc2;
	};
	struct Ch1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ch1;
	};
	struct Ch1n
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ch1n;
	};
	struct Ch2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ch2;
	};
	struct Ch2n
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ch2n;
	};
	struct Ch3
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ch3;
	};
	struct Ch3n
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ch3n;
	};
	struct Ch4
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ch4;
	};
	struct Ch4n
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ch4n;
	};
	struct Cha1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Cha1;
	};
	struct Cha2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Cha2;
	};
	struct Chb1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Chb1;
	};
	struct Chb2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Chb2;
	};
	struct Chc1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Chc1;
	};
	struct Chc2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Chc2;
	};
	struct Chd1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Chd1;
	};
	struct Chd2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Chd2;
	};
	struct Ck
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ck;
	};
	struct Ck1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ck1;
	};
	struct Ck2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ck2;
	};
	struct Ckin
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ckin;
	};
	struct Clk
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Clk;
	};
	struct Crssync
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Crssync;
	};
	struct Cts
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Cts;
	};
	struct D1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::D1;
	};
	struct D2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::D2;
	};
	struct Dbcc1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Dbcc1;
	};
	struct Dbcc2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Dbcc2;
	};
	struct De
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::De;
	};
	struct Dm
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Dm;
	};
	struct Dp
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Dp;
	};
	struct Eev3
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Eev3;
	};
	struct Eev4
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Eev4;
	};
	struct Eev5
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Eev5;
	};
	struct Eev6
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Eev6;
	};
	struct Eev7
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Eev7;
	};
	struct Eev8
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Eev8;
	};
	struct Eev9
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Eev9;
	};
	struct Etr
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Etr;
	};
	struct Flt1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Flt1;
	};
	struct Flt2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Flt2;
	};
	struct Flt3
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Flt3;
	};
	struct Flt4
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Flt4;
	};
	struct Flt5
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Flt5;
	};
	struct Frstx1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Frstx1;
	};
	struct Frstx2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Frstx2;
	};
	struct Fsa
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Fsa;
	};
	struct Fsb
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Fsb;
	};
	struct In1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::In1;
	};
	struct In10
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::In10;
	};
	struct In11
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::In11;
	};
	struct In12
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::In12;
	};
	struct In13
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::In13;
	};
	struct In14
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::In14;
	};
	struct In15
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::In15;
	};
	struct In17
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::In17;
	};
	struct In2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::In2;
	};
	struct In3
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::In3;
	};
	struct In4
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::In4;
	};
	struct In5
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::In5;
	};
	struct Inm
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inm;
	};
	struct Inp
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inp;
	};
	struct Jtck
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Jtck;
	};
	struct Jtdi
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Jtdi;
	};
	struct Jtdo
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Jtdo;
	};
	struct Jtms
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Jtms;
	};
	struct Jtrst
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Jtrst;
	};
	struct Lsco
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Lsco;
	};
	struct Mck
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Mck;
	};
	struct Mclka
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Mclka;
	};
	struct Mclkb
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Mclkb;
	};
	struct Mco
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Mco;
	};
	struct Miso
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Miso;
	};
	struct Mosi
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Mosi;
	};
	struct Nss
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Nss;
	};
	struct Osc32in
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Osc32in;
	};
	struct Osc32out
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Osc32out;
	};
	struct Oscin
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Oscin;
	};
	struct Oscout
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Oscout;
	};
	struct Out
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Out;
	};
	struct Out1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Out1;
	};
	struct Out2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Out2;
	};
	struct Pvdin
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Pvdin;
	};
	struct Refin
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Refin;
	};
	struct Rts
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Rts;
	};
	struct Rx
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Rx;
	};
	struct Scin
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Scin;
	};
	struct Sck
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Sck;
	};
	struct Scka
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Scka;
	};
	struct Sckb
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Sckb;
	};
	struct Scl
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Scl;
	};
	struct Scout
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Scout;
	};
	struct Sd
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Sd;
	};
	struct Sda
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Sda;
	};
	struct Sdb
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Sdb;
	};
	struct Smba
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Smba;
	};
	struct Swclk
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Swclk;
	};
	struct Swdio
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Swdio;
	};
	struct Swo
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Swo;
	};
	struct Tamp1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Tamp1;
	};
	struct Tamp2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Tamp2;
	};
	struct Ts
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ts;
	};
	struct Tx
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Tx;
	};
	struct Vinm
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Vinm;
	};
	struct Vinm0
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Vinm0;
	};
	struct Vinm1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Vinm1;
	};
	struct Vinmsec
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Vinmsec;
	};
	struct Vinp
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Vinp;
	};
	struct Vinpsec
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Vinpsec;
	};
	struct Vout
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Vout;
	};
	struct Wkup1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Wkup1;
	};
	struct Wkup2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Wkup2;
	};
	struct Wkup4
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Wkup4;
	};
	struct Ws
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ws;
	};
};

} // namespace modm::platform
