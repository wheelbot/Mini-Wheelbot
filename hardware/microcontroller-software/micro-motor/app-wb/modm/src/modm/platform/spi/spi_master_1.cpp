/*
 * Copyright (c) 2009, Martin Rosekeit
 * Copyright (c) 2009-2012, Fabian Greif
 * Copyright (c) 2010, Georgi Grinshpun
 * Copyright (c) 2012-2017, Niklas Hauser
 * Copyright (c) 2013, Kevin Läufer
 * Copyright (c) 2014, Sascha Schade
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "spi_master_1.hpp"

modm::ResumableResult<uint8_t>
modm::platform::SpiMaster1::transfer(uint8_t data)
{
	// wait for previous transfer to finish
	while(!SpiHal1::isTransmitRegisterEmpty())
		modm::this_fiber::yield();

	// start transfer by copying data into register
	SpiHal1::write(data);

	// wait for current transfer to finish
	while(!SpiHal1::isReceiveRegisterNotEmpty())
		modm::this_fiber::yield();

	// read the received byte
	SpiHal1::read(data);

	return data;
}

modm::ResumableResult<void>
modm::platform::SpiMaster1::transfer(
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
