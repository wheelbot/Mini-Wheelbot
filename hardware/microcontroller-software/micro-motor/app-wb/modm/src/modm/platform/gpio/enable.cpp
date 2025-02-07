/*
 * Copyright (c) 2013-2014, Kevin Läufer
 * Copyright (c) 2013-2018, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "../device.hpp"
#include <modm/platform/core/hardware_init.hpp>

void
modm_gpio_enable(void)
{
	// Enable GPIO clock
	RCC->AHB2ENR  |=
		RCC_AHB2ENR_GPIOAEN |
		RCC_AHB2ENR_GPIOBEN |
		RCC_AHB2ENR_GPIOCEN |
		RCC_AHB2ENR_GPIOFEN |
		RCC_AHB2ENR_GPIOGEN;
	// Reset GPIO peripheral
	RCC->AHB2RSTR |=
		RCC_AHB2RSTR_GPIOARST |
		RCC_AHB2RSTR_GPIOBRST |
		RCC_AHB2RSTR_GPIOCRST |
		RCC_AHB2RSTR_GPIOFRST |
		RCC_AHB2RSTR_GPIOGRST;
	RCC->AHB2RSTR &= ~(
		RCC_AHB2RSTR_GPIOARST |
		RCC_AHB2RSTR_GPIOBRST |
		RCC_AHB2RSTR_GPIOCRST |
		RCC_AHB2RSTR_GPIOFRST |
		RCC_AHB2RSTR_GPIOGRST);
}

MODM_HARDWARE_INIT_ORDER(modm_gpio_enable, 80);
