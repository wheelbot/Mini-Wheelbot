/*
 * Copyright (c) 2015-2017, Niklas Hauser
 * Copyright (c) 2017, Fabian Greif
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_ADC_INTERRUPT_4_HPP
#define MODM_STM32_ADC_INTERRUPT_4_HPP

#include "adc_4.hpp"
#include <modm/architecture/interface/adc_interrupt.hpp>


namespace modm
{

namespace platform
{

/**
 * ADC Interrupt module
 *
 * This class allows you to attach functions to the ADC Conversion
 * Complete Interrupt via function pointers.
 * Be aware however, that this implementation is slower and requires
 * more resources than writing the function code directly into
 * the interrupt service routines.
 *
 * @see AnalogSensors uses this implemenation.
 *
 * @ingroup		modm_platform_adc_4
 * @author		Niklas Hauser
 */
class AdcInterrupt4 : public Adc4, public modm::AdcInterrupt
{
public:
	static inline void
	attachInterruptHandler(Handler handler)
	{
		AdcInterrupt4::handler = handler;
	}

	static Handler handler;
};

}	// namespace platform

}	// namespace modm

#endif // MODM_STM32_ADC_INTERRUPT_HPP
