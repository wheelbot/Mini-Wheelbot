/*
 * Copyright (c) 2020, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "flash.hpp"

static constexpr uint32_t FLASH_SR_ERR = 0xfffe;

namespace modm::platform
{

bool
Flash::unlock()
{
	Flash::enable();
	if (isLocked())
	{
		FLASH->KEYR = 0x45670123;
		FLASH->KEYR = 0xCDEF89AB;
	}
	return not isLocked();
}

uint8_t
Flash::getPage(uintptr_t offset)
{
	const uint8_t index = (offset >> (FLASH->OPTR & FLASH_OPTR_DBANK ? 11 : 12));
	return index;
}

uint32_t
Flash::getOffset(uint8_t index)
{
	return (1ul << (FLASH->OPTR & FLASH_OPTR_DBANK ? 11 : 12)) * index;
}

size_t
Flash::getSize([[maybe_unused]] uint8_t index)
{
	return (1ul	<< (FLASH->OPTR & FLASH_OPTR_DBANK ? 11 : 12));
}

modm_ramcode uint32_t
Flash::erase(uint8_t index)
{
	FLASH->SR = FLASH_SR_ERR;
	uint32_t cr = FLASH_CR_STRT | FLASH_CR_PER;
	uint32_t page = index;
	if (FLASH->OPTR & FLASH_OPTR_DBANK and index >= (Size/2 >> 11))
	{
		// second bank index starts back at zero again
		page -= (Size/2 >> 11);
		cr |= FLASH_CR_BKER;
	}
	FLASH->CR = cr | ((page << FLASH_CR_PNB_Pos) & FLASH_CR_PNB_Msk);
	while(isBusy()) ;
	FLASH->CR = 0;
	return FLASH->SR & FLASH_SR_ERR;
}

modm_ramcode uint32_t
Flash::program(uintptr_t addr, MaxWordType data)
{
	FLASH->SR = FLASH_SR_ERR;
	FLASH->CR = FLASH_CR_PG;
	*(uint64_t*) addr = data;
	while(isBusy()) ;

	FLASH->SR |= FLASH_SR_EOP;
	FLASH->CR = 0;
	return FLASH->SR & FLASH_SR_ERR;
}

} // namespace modm::platform
