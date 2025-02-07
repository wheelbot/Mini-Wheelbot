/*
 * Copyright (c) 2009, Martin Rosekeit
 * Copyright (c) 2009-2012, Fabian Greif
 * Copyright (c) 2011, Georgi Grinshpun
 * Copyright (c) 2012, 2016, Sascha Schade
 * Copyright (c) 2012, 2014-2019, 2021, Niklas Hauser
 * Copyright (c) 2013-2014, Kevin Läufer
 * Copyright (c) 2018, 2021, Christopher Durand
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "rcc.hpp"

// CMSIS Core compliance
constinit uint32_t modm_fastdata SystemCoreClock(modm::platform::Rcc::BootFrequency);
modm_weak void SystemCoreClockUpdate() { /* Nothing to update */ }

namespace modm::platform
{
constinit uint16_t modm_fastdata delay_fcpu_MHz(computeDelayMhz(Rcc::BootFrequency));
constinit uint16_t modm_fastdata delay_ns_per_loop(computeDelayNsPerLoop(Rcc::BootFrequency));

// ----------------------------------------------------------------------------
bool
Rcc::enableInternalClockMHz48(uint32_t waitCycles)
{
	bool retval;
	RCC->CRRCR |= RCC_CRRCR_HSI48ON;
	while (not (retval = (RCC->CRRCR & RCC_CRRCR_HSI48RDY)) and --waitCycles)
		;
	return retval;
}
bool
Rcc::enableInternalClock(uint32_t waitCycles)
{
	bool retval;
	RCC->CR |= RCC_CR_HSION;
	while (not (retval = (RCC->CR & RCC_CR_HSIRDY)) and --waitCycles)
		;
	return retval;
}

bool
Rcc::enableExternalClock(uint32_t waitCycles)
{
	bool retval;
	RCC->CR |= RCC_CR_HSEBYP | RCC_CR_HSEON;
	while (not (retval = (RCC->CR & RCC_CR_HSERDY)) and --waitCycles)
		;
	return retval;
}

bool
Rcc::enableExternalCrystal(uint32_t waitCycles)
{
	bool retval;
	RCC->CR = (RCC->CR & ~RCC_CR_HSEBYP) | RCC_CR_HSEON;
	while (not (retval = (RCC->CR & RCC_CR_HSERDY)) and --waitCycles)
		;
	return retval;
}


bool
Rcc::enableLowSpeedInternalClock(uint32_t waitCycles)
{
	bool retval;
	RCC->CSR |= RCC_CSR_LSION;
	while (not (retval = (RCC->CSR & RCC_CSR_LSIRDY)) and --waitCycles)
		;
	return retval;
}

bool
Rcc::enableLowSpeedExternalClock(uint32_t waitCycles)
{
	bool retval;
	RCC->BDCR |= RCC_BDCR_LSEBYP | RCC_BDCR_LSEON;
	while (not (retval = (RCC->BDCR & RCC_BDCR_LSERDY)) and --waitCycles)
		;
	return retval;
}

bool
Rcc::enableLowSpeedExternalCrystal(uint32_t waitCycles)
{
	bool retval;
	RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_LSEBYP) | RCC_BDCR_LSEON;
	while (not (retval = (RCC->BDCR & RCC_BDCR_LSERDY)) and --waitCycles)
		;
	return retval;
}

bool
Rcc::enablePll(PllSource source, const PllFactors& pllFactors, uint32_t waitCycles)
{
	// Read reserved values and clear all other values
	uint32_t tmp = RCC->PLLCFGR & ~(
			RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN |
			// RCC_PLLCFGR_PLLPEN | RCC_PLLCFGR_PLLP |
			RCC_PLLCFGR_PLLREN | RCC_PLLCFGR_PLLR);

	// PLLSRC source for pll
	tmp |= static_cast<uint32_t>(source);

	// PLLM factor is user defined VCO input frequency must be configured between 4MHz and 16Mhz
	tmp |= (uint32_t(pllFactors.pllM - 1) << RCC_PLLCFGR_PLLM_Pos) & RCC_PLLCFGR_PLLM;

	// PLLN factor is user defined: between 64 and 344 MHz
	tmp |= (uint32_t(pllFactors.pllN) << RCC_PLLCFGR_PLLN_Pos) & RCC_PLLCFGR_PLLN;

	// PLLR divider for CPU frequency
	tmp |= ((uint32_t(pllFactors.pllR / 2) - 1) << RCC_PLLCFGR_PLLR_Pos) & RCC_PLLCFGR_PLLR;
	// PLLQ (21) divider for USB frequency; (00: PLLQ = 2, 01: PLLQ = 4, etc.)
	if (pllFactors.pllQ != 0xff) {
		tmp &= ~RCC_PLLCFGR_PLLQ;
		tmp |= (((uint32_t) (pllFactors.pllQ / 2) - 1) << RCC_PLLCFGR_PLLQ_Pos) & RCC_PLLCFGR_PLLQ;
		// enable pll USB clock output
		tmp |= RCC_PLLCFGR_PLLQEN;
	}
	// enable pll CPU clock output
	tmp |= RCC_PLLCFGR_PLLREN;

	RCC->PLLCFGR = tmp;

	// enable pll
	RCC->CR |= RCC_CR_PLLON;

	while (not (tmp = (RCC->CR & RCC_CR_PLLRDY)) and --waitCycles)
		;

	return tmp;
}

bool
Rcc::disablePll(uint32_t waitCycles)
{
	RCC->CR &= ~RCC_CR_PLLON;
	while ((RCC->CR & RCC_CR_PLLRDY) and --waitCycles)
		;
	return waitCycles > 0;
}

bool
Rcc::setVoltageScaling(VoltageScaling voltage, uint32_t waitCycles)
{
	const auto currentSetting = PWR->CR1 & PWR_CR1_VOS;
	if (voltage == VoltageScaling::Boost) {
		PWR->CR5 &= ~PWR_CR5_R1MODE;
	} else {
		PWR->CR5 |= PWR_CR5_R1MODE;
	}
	if (voltage != VoltageScaling::Scale2) {
		if(static_cast<VoltageScaling>(currentSetting) == VoltageScaling::Scale2) {
			PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | PWR_CR1_VOS_0;
			while (PWR->SR2 & PWR_SR2_VOSF)
				if (--waitCycles == 0) return false;
		}
	} else {
		PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | PWR_CR1_VOS_1;
	}

	return true;
}
bool
Rcc::enableSystemClock(SystemClockSource src, uint32_t waitCycles)
{
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | uint32_t(src);

	// Wait till the main PLL is used as system clock source
	src = SystemClockSource(uint32_t(src) << RCC_CFGR_SWS_Pos);
	while ((RCC->CFGR & RCC_CFGR_SWS) != uint32_t(src))
		if (not --waitCycles) return false;

	return true;
}

bool
Rcc::setCanPrescaler(CanPrescaler prescaler)
{
	enable<Peripheral::Fdcan1>();

	// FDCAN1 must enter initialization mode to configure common divider.
	// This will stop operation of FDCAN1.
	// The setting only takes effect after resetting INIT in FDCAN1_CCCR
	if (FDCAN_CONFIG->CKDIV != uint32_t(prescaler)) {
		FDCAN1->CCCR = FDCAN_CCCR_INIT;

		// Wait until the initialization mode is entered
		int deadlockPreventer = 10'000; // max ~10ms
		while (((FDCAN1->CCCR & FDCAN_CCCR_INIT) == 0) and (deadlockPreventer-- > 0)) {
			modm::delay_us(1);
		}

		if(deadlockPreventer == 0) {
			return false;
		}

		FDCAN1->CCCR |= FDCAN_CCCR_CCE;
		FDCAN_CONFIG->CKDIV = uint32_t(prescaler);

		FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
	}
	return true;
}
}
