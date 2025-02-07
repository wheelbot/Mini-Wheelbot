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

#include "dac_2.hpp"
namespace modm::platform
{

void Dac2::enableChannel(Channel chan)
{
	switch(chan) {
	case Channel::Channel1:
		DAC2->CR |= DAC_CR_EN1_Msk;
		break;
	}
}

void Dac2::disableChannel(Channel chan)
{
	switch(chan) {
	case Channel::Channel1:
		DAC2->CR &= ~DAC_CR_EN1_Msk;
		break;
	}
}

void Dac2::setMode(Channel channel, Mode mode)
{
	static_assert(DAC_MCR_MODE1_Pos == 0);
	if(channel == Channel::Channel1) {
		DAC2->CR &= ~DAC_CR_EN1;
	} else {
	}

	const auto shift = 0;
	const auto clearMask = ~(DAC_MCR_MODE1_Msk << shift);
	DAC2->MCR = (DAC2->MCR & clearMask) | (uint32_t(mode) << shift);
}

} // namespace modm::platform
