/*
 * Copyright (c) 2013, Kevin Läufer
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

#include "uart_spi_master_3.hpp"
#include <modm/math/utils/bit_operation.hpp>

modm::platform::UartSpiMaster3::DataOrder
	modm::platform::UartSpiMaster3::dataOrder =
	modm::platform::UartSpiMaster3::DataOrder::MsbFirst;

// Bit0: single transfer state
// Bit1: block transfer state
uint8_t
modm::platform::UartSpiMaster3::state(0);

// ----------------------------------------------------------------------------

modm::ResumableResult<uint8_t>
modm::platform::UartSpiMaster3::transfer(uint8_t data)
{
	// wait for previous transfer to finish
	while (!UsartHal3::isTransmitRegisterEmpty())
		modm::this_fiber::yield();

	// start transfer by copying data into register
	if(dataOrder == DataOrder::MsbFirst)
		data = ::modm::bitReverse(data);
	UsartHal3::write(data);

	// wait for current transfer to finish
	while (!UsartHal3::isReceiveRegisterNotEmpty())
		modm::this_fiber::yield();

	UsartHal3::read(data);

	if(dataOrder == DataOrder::MsbFirst)
		data = ::modm::bitReverse(data);

	return data;
}

modm::ResumableResult<void>
modm::platform::UartSpiMaster3::transfer(
		const uint8_t * tx, uint8_t * rx, std::size_t length)
{
	for (std::size_t index = 0; index < length; index++)
	{
		// if tx == 0, we use a dummy byte 0x00 else we copy it from the array
		auto result = transfer(tx ? tx[index] : 0);
		// if rx != 0, we copy the result into the array
		if (rx) rx[index] = result;
	}
}
