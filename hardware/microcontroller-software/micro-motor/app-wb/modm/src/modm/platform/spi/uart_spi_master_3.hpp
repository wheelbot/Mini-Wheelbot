/*
 * Copyright (c) 2013, 2016, Kevin Läufer
 * Copyright (c) 2013-2018, Niklas Hauser
 * Copyright (c) 2014, Sascha Schade
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_UART_SPI_MASTER3_HPP
#define MODM_STM32_UART_SPI_MASTER3_HPP

#include <modm/architecture/interface/spi_lock.hpp>
#include <modm/architecture/interface/spi_master.hpp>
#include <modm/platform/gpio/connector.hpp>
#include "../uart/uart_hal_3.hpp"

namespace modm
{

namespace platform
{

/**
 * Serial Peripheral Interface of the Uart module.
 *
 * @warning	Be aware, that the UART module can send bytes only LSB first.
 *			Therefore the data is bit reversed in software for DataOrder::MsbFirst.
 *
 * @author		Niklas Hauser
 * @ingroup		modm_platform_uart_spi modm_platform_uart_spi_3
 */
class UartSpiMaster3 : public modm::SpiMaster, public UartBase,
                              public SpiLock<UartSpiMaster3>
{
	static DataOrder dataOrder;
	static uint8_t state;
	static uint8_t count;
	static void *context;
	static ConfigurationHandler configuration;
public:
	enum class
	DataMode : uint32_t
	{
		Mode0 = 0b00,			///< clock normal,   sample on rising  edge
		Mode1 = USART_CR2_CPHA,	///< clock normal,   sample on falling edge
		Mode2 = USART_CR2_CPOL,	///< clock inverted, sample on falling  edge
		Mode3 = USART_CR2_CPOL | USART_CR2_CPHA
		///< clock inverted, sample on rising edge
	};

public:
	template< class... Signals >
	static void
	connect()
	{
		using Connector = GpioConnector<Peripheral::Usart3, Signals...>;
		using Sck = typename Connector::template GetSignal<Gpio::Signal::Ck>;
		using Mosi = typename Connector::template GetSignal<Gpio::Signal::Tx>;
		using Miso = typename Connector::template GetSignal<Gpio::Signal::Rx>;

		// Connector::disconnect();
		Sck::setOutput(Gpio::OutputType::PushPull);
		Mosi::setOutput(Gpio::OutputType::PushPull);
		Miso::setInput(Gpio::InputType::Floating);
		Connector::connect();
	}

	template< 	class SystemClock, baudrate_t baudrate, percent_t tolerance=pct(5) >
	static void
	initialize()
	{
		UsartHal3::initialize<SystemClock, baudrate, tolerance>(
				UsartHal3::Parity::Disabled, UsartHal3::WordLength::Bit8);
		UsartHal3::setSpiClock(UsartHal3::SpiClock::Enabled,
		                              UsartHal3::LastBitClockPulse::Output);
		UsartHal3::setTransmitterEnable(true);
		UsartHal3::setReceiverEnable(true);
		UsartHal3::enableOperation();
		dataOrder = DataOrder::MsbFirst;
		state = 0;
	}


	static void
	setDataMode(DataMode mode)
	{
		if constexpr (UsartHal3::isExtended) UsartHal3::disableOperation();
		UsartHal3::setSpiDataMode(static_cast<UartBase::SpiDataMode>(mode));
		if constexpr (UsartHal3::isExtended) UsartHal3::enableOperation();
	}

	/// @warning 	On this target, only `DataOrder::LsbFirst` is natively
	///				supported!
	///				`DataOrder::MsbFirst` is reimplemented in software using
	///				modm::bitReverse(), which adds some overhead.
	/// @see	modm::bitReverse()
	static void
	setDataOrder(DataOrder order)
	{
		dataOrder = order;
	}

	static uint8_t
	transferBlocking(uint8_t data)
	{
		return RF_CALL_BLOCKING(transfer(data));
	}

	static void
	transferBlocking(const uint8_t *tx, uint8_t *rx, std::size_t length)
	{
		RF_CALL_BLOCKING(transfer(tx, rx, length));
	}


	static modm::ResumableResult<uint8_t>
	transfer(uint8_t data);

	static modm::ResumableResult<void>
	transfer(const uint8_t *tx, uint8_t *rx, std::size_t length);
};

} // namespace platform

} // namespace modm

#endif // MODM_STM32_UART_SPI_MASTER3_HPP
