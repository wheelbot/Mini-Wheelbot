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

#ifndef MODM_STM32_COMP2_HPP
#define MODM_STM32_COMP2_HPP

#include "base.hpp"

#include <modm/platform/gpio/connector.hpp>

namespace modm::platform
{
	/**
	 * @brief		Comparator Class for STM32
	 *
	 * @internal
	 * @ingroup		modm_platform_comp modm_platform_comp_2
	 */
	class Comp2 : public CompBase
	{
	public:
		enum class
		InvertingInput
		{
			Vref1Div4		= (0b000 << 4) | (0b11 << 22),
			Vref1Div2		= (0b001 << 4) | (0b11 << 22),
			Vref3Div4		= (0b010 << 4) | (0b11 << 22),
			Vref			= (0b011 << 4) | (0b10 << 22),
			Dac3Ch2			= 0b100 << 4,
			Dac1Ch2			= 0b101 << 4,
			GpioA5			= 0b110 << 4,
			GpioA2			= 0b111 << 4,
			};
	protected:
		static constexpr uint32_t InvertingInputMask = (0b1111 << 4) | (0b11 << 22);
		public:
		enum class
		NonInvertingInput
		{
			GpioA7			= 0b0 << 8,
			GpioA3			= 0b1 << 8,
			};
	protected:
		static constexpr uint32_t NonInvertingInputMask = 0b1 << 8;
		public:
		enum class
		BlankingSource
		{
			NoBlanking		= 0b000 << 18,

			Tim1Oc5			= 0b001 << COMP_CSR_BLANKING_Pos,
			Tim2Oc3			= 0b010 << COMP_CSR_BLANKING_Pos,
			Tim3Oc3			= 0b011 << COMP_CSR_BLANKING_Pos,
			Tim8Oc5			= 0b100 << COMP_CSR_BLANKING_Pos,
			Tim20Oc5			= 0b101 << COMP_CSR_BLANKING_Pos,
			Tim15Oc1			= 0b110 << COMP_CSR_BLANKING_Pos,
			Tim4Oc3			= 0b111 << COMP_CSR_BLANKING_Pos,
			};
	protected:
		static constexpr uint32_t BlankingSourceMask = COMP_CSR_BLANKING_Msk;

	public:
		/**
		 * Initialize and enable the comparator.
		 *
		 * Enables the comperator and sets important values.
		 *
		 * Do NOT set lock = true if you want to be able to set other values
		 * later.
		 */
		static inline void
		initialize(
					InvertingInput n_in,
				NonInvertingInput p_in,
				Hysteresis hyst = Hysteresis::NoHysteresis,
				Polarity pol = Polarity::NonInverted,
					bool lock_comp = false)
		{
			setInvertingInput(n_in);
		setNonInvertingInput(p_in);
		setHysteresis(hyst);
		setPolarity(pol);
			setEnabled(true);	// enable comparator
			if(lock_comp) {
				lock();
			}
		}

		/**
		 * \brief	Enable/Disable the comparator.
		 */
		static inline void
		setEnabled(bool enabled)
		{
			if(enabled)
				COMP2->CSR |= COMP_CSR_EN;
			else
				COMP2->CSR &= ~COMP_CSR_EN;
		}

		/**
		 * \brief	Returns whether the comparator is enabled.
		 */
		static inline bool
		isEnabled()
		{
			return COMP2->CSR & COMP_CSR_EN;
		}

	/**
		 * \brief	Selects what the inverting input is connected to.
		 */
		static inline void
		setInvertingInput(InvertingInput input)
		{
			COMP2->CSR = (COMP2->CSR & ~InvertingInputMask) | static_cast<uint32_t>(input);
		}

		/**.
		 * \brief	Returns what is connected to the inverting input.
		 */
		static inline InvertingInput
		getInvertingInput()
		{
			return static_cast<InvertingInput>(COMP2->CSR & InvertingInputMask);
		}

	/**
		 * \brief	Selects what the non-inverting input is connected to.
		 */
		static inline void
		setNonInvertingInput(NonInvertingInput input)
		{
			COMP2->CSR = (COMP2->CSR & ~NonInvertingInputMask) | static_cast<uint32_t>(input);
		}

		/**.
		 * \brief	Returns what is connected to the non-inverting input.
		 */
		static inline NonInvertingInput
		getNonInvertingInput()
		{
			return static_cast<NonInvertingInput>(COMP2->CSR & NonInvertingInputMask);
		}
	/**
		 * \brief	Selects output polarity.
		 */
		static inline void
		setPolarity(Polarity pol)
		{
			COMP2->CSR = (COMP2->CSR & ~PolarityMask) | static_cast<uint32_t>(pol);
		}

		/**.
		 * \brief	Returns output polarity.
		 */
		static inline Polarity
		getPolarity()
		{
			return static_cast<Polarity>(COMP2->CSR & PolarityMask);
		}

		/**
		 * \brief	Selects the hysteresis.
		 */
		static inline void
		setHysteresis(Hysteresis hyst)
		{
			COMP2->CSR = (COMP2->CSR & ~HysteresisMask) | static_cast<uint32_t>(hyst);
		}

		/**.
		 * \brief	Returns the hysteresis.
		 */
		static inline Hysteresis
		getHysteresis()
		{
			return static_cast<Hysteresis> (COMP2->CSR & HysteresisMask);
		}

		/**
		 * \brief	Selects the blanking source.
		 */
		static inline void
		setBlankingSource(BlankingSource blanking)
		{
			COMP2->CSR |= (COMP2->CSR & ~BlankingSourceMask) | static_cast<uint32_t>(blanking);
		}

		/**.
		 * \brief	Returns the blanking source.
		 */
		static inline BlankingSource
		getBlankingSource()
		{
			return static_cast<BlankingSource>(COMP2->CSR & BlankingSourceMask);
		}

		/**
		 * \brief	Returns the current Comparator output.
		 */
		static inline bool
		getOutput()
		{
		return COMP2->CSR & COMP_CSR_VALUE;
		}

		/**
		 * \brief	Locks the comparator.
		 *
		 * Comparator can only be unlocked by a system reset.
		 */
		static inline void
		lock(void)
		{
			COMP2->CSR |= COMP_CSR_LOCK;
		}

		/**
		 * \brief	Returns true if the comparator is locked.
		 */
		static inline bool
		isLocked(void)
		{
			return COMP2->CSR & COMP_CSR_LOCK;
		}

	public:
		// start inherited documentation
		template< class... Signals >
		static void
		connect()
		{
			using Connector = GpioConnector<Peripheral::Comp2, Signals...>;
			Connector::connect();
		}
	};
}

#endif	//  MODM_STM32_COMP2_HPP
