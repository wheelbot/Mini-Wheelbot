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

#include "dac_1.hpp"
namespace modm::platform
{

void Dac1::enableChannel(Channel chan)
{
	switch(chan) {
	case Channel::Channel1:
		DAC1->CR |= DAC_CR_EN1_Msk;
		break;
	case Channel::Channel2:
		DAC1->CR |= DAC_CR_EN2_Msk;
		break;
	}
}

void Dac1::disableChannel(Channel chan)
{
	switch(chan) {
	case Channel::Channel1:
		DAC1->CR &= ~DAC_CR_EN1_Msk;
		break;
	case Channel::Channel2:
		DAC1->CR &= ~DAC_CR_EN2_Msk;
		break;
	}
}

void Dac1::setMode(Channel channel, Mode mode)
{
	static_assert(DAC_MCR_MODE1_Pos == 0);
	static_assert(DAC_MCR_MODE2_Pos == 16);
	if(channel == Channel::Channel1) {
		DAC1->CR &= ~DAC_CR_EN1;
	} else {
		DAC1->CR &= ~DAC_CR_EN2;
	}

	const auto shift = (channel == Channel::Channel1) ? 0 : DAC_MCR_MODE2_Pos;
	const auto clearMask = ~(DAC_MCR_MODE1_Msk << shift);
	DAC1->MCR = (DAC1->MCR & clearMask) | (uint32_t(mode) << shift);
}

} // namespace modm::platform
