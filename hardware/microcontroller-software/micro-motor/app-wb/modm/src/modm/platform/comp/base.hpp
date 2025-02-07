/*
 * Copyright (c) 2018, Raphael Lehmann
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_COMP_BASE_HPP
#define MODM_STM32_COMP_BASE_HPP

#include <modm/platform/device.hpp>

namespace modm::platform
{
	/// @ingroup modm_platform_comp
	class CompBase
	{
	public:
		enum class
		Polarity
		{
			NonInverted	= 0b0 << 15,
			Inverted	= 0b1 << 15,
		};
	protected:
		static constexpr uint32_t PolarityMask = 0b1 << 15;

	public:
		enum class
		Hysteresis
		{
			NoHysteresis		= 0b000 << COMP_CSR_HYST_Pos,
			Hysteresis10mV		= 0b001 << COMP_CSR_HYST_Pos,
			Hysteresis20mV		= 0b010 << COMP_CSR_HYST_Pos,
			Hysteresis30mV		= 0b011 << COMP_CSR_HYST_Pos,
			Hysteresis40mV		= 0b100 << COMP_CSR_HYST_Pos,
			Hysteresis50mV		= 0b101 << COMP_CSR_HYST_Pos,
			Hysteresis60mV		= 0b110 << COMP_CSR_HYST_Pos,
			Hysteresis70mV		= 0b111 << COMP_CSR_HYST_Pos,

			// for compatibility:
			LowHysteresis		= Hysteresis10mV,
			MediumHysteresis	= Hysteresis40mV,
			HighHysteresis		= Hysteresis70mV,
		};
	protected:
		static constexpr uint32_t HysteresisMask = COMP_CSR_HYST_Msk;
	};
}

#endif	//  MODM_STM32_COMP_BASE_HPP
