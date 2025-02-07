/*
 * Copyright (c) 2019, 2021, 2024, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

namespace modm::platform
{

constexpr Rcc::flash_latency
Rcc::computeFlashLatency(uint32_t Core_Hz, uint16_t Core_mV)
{
	constexpr uint32_t flash_latency_1000[] =
	{
		8000000,
		16000000,
		26000000,
	};
	constexpr uint32_t flash_latency_1280[] =
	{
		20000000,
		40000000,
		60000000,
		80000000,
		100000000,
		120000000,
		140000000,
		160000000,
		170000000,
	};
	const uint32_t *lut(flash_latency_1000);
	uint8_t lut_size(sizeof(flash_latency_1000) / sizeof(uint32_t));
	// find the right table for the voltage
	if (1280 <= Core_mV) {
		lut = flash_latency_1280;
		lut_size = sizeof(flash_latency_1280) / sizeof(uint32_t);
	}
	// find the next highest frequency in the table
	uint8_t latency(0);
	uint32_t max_freq(0);
	while (latency < lut_size)
	{
		if (Core_Hz <= (max_freq = lut[latency]))
			break;
		latency++;
	}
	return {latency, max_freq};
}

template< uint32_t Core_Hz, uint16_t Core_mV>
uint32_t
Rcc::setFlashLatency()
{
	constexpr flash_latency fl = computeFlashLatency(Core_Hz, Core_mV);
	static_assert(Core_Hz <= fl.max_frequency, "CPU Frequency is too high for this core voltage!");

	uint32_t acr = FLASH->ACR & ~FLASH_ACR_LATENCY;
	// set flash latency
	acr |= fl.latency;
	// enable flash prefetch and data and instruction cache
	acr |= FLASH_ACR_PRFTEN | FLASH_ACR_DCEN | FLASH_ACR_ICEN;
	FLASH->ACR = acr;
	__DSB(); __ISB();
	return fl.max_frequency;
}

template< uint32_t Core_Hz >
void
Rcc::updateCoreFrequency()
{
	SystemCoreClock = Core_Hz;
	delay_fcpu_MHz = computeDelayMhz(Core_Hz);
	delay_ns_per_loop = computeDelayNsPerLoop(Core_Hz);
}

constexpr bool
rcc_check_enable(Peripheral peripheral)
{
	switch(peripheral) {
		case Peripheral::Cordic:
		case Peripheral::Crc:
		case Peripheral::Dac1:
		case Peripheral::Dac2:
		case Peripheral::Dac3:
		case Peripheral::Dac4:
		case Peripheral::Dma1:
		case Peripheral::Dma2:
		case Peripheral::Dmamux1:
		case Peripheral::Fdcan1:
		case Peripheral::Flash:
		case Peripheral::Fmac:
		case Peripheral::Hrtim1:
		case Peripheral::I2c1:
		case Peripheral::I2c2:
		case Peripheral::I2c3:
		case Peripheral::I2c4:
		case Peripheral::Lptim1:
		case Peripheral::Lpuart1:
		case Peripheral::Rng:
		case Peripheral::Rtc:
		case Peripheral::Sai1:
		case Peripheral::Spi1:
		case Peripheral::Spi2:
		case Peripheral::Spi3:
		case Peripheral::Tim1:
		case Peripheral::Tim15:
		case Peripheral::Tim16:
		case Peripheral::Tim17:
		case Peripheral::Tim2:
		case Peripheral::Tim20:
		case Peripheral::Tim3:
		case Peripheral::Tim4:
		case Peripheral::Tim5:
		case Peripheral::Tim6:
		case Peripheral::Tim7:
		case Peripheral::Tim8:
		case Peripheral::Ucpd1:
		case Peripheral::Usart1:
		case Peripheral::Usart2:
		case Peripheral::Usart3:
		case Peripheral::Usb:
		case Peripheral::Wwdg:
			return true;
		default:
			return false;
	}
}

template< Peripheral peripheral >
void
Rcc::enable()
{
	static_assert(rcc_check_enable(peripheral),
		"Rcc::enable() doesn't know this peripheral!");

	__DSB();
	if constexpr (peripheral == Peripheral::Cordic)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN; __DSB();
			RCC->AHB1RSTR |= RCC_AHB1RSTR_CORDICRST; __DSB();
			RCC->AHB1RSTR &= ~RCC_AHB1RSTR_CORDICRST;
		}
	if constexpr (peripheral == Peripheral::Crc)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN; __DSB();
			RCC->AHB1RSTR |= RCC_AHB1RSTR_CRCRST; __DSB();
			RCC->AHB1RSTR &= ~RCC_AHB1RSTR_CRCRST;
		}
	if constexpr (peripheral == Peripheral::Dac1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN; __DSB();
			RCC->AHB2RSTR |= RCC_AHB2RSTR_DAC1RST; __DSB();
			RCC->AHB2RSTR &= ~RCC_AHB2RSTR_DAC1RST;
		}
	if constexpr (peripheral == Peripheral::Dac2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB2ENR |= RCC_AHB2ENR_DAC2EN; __DSB();
			RCC->AHB2RSTR |= RCC_AHB2RSTR_DAC2RST; __DSB();
			RCC->AHB2RSTR &= ~RCC_AHB2RSTR_DAC2RST;
		}
	if constexpr (peripheral == Peripheral::Dac3)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB2ENR |= RCC_AHB2ENR_DAC3EN; __DSB();
			RCC->AHB2RSTR |= RCC_AHB2RSTR_DAC3RST; __DSB();
			RCC->AHB2RSTR &= ~RCC_AHB2RSTR_DAC3RST;
		}
	if constexpr (peripheral == Peripheral::Dac4)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB2ENR |= RCC_AHB2ENR_DAC4EN; __DSB();
			RCC->AHB2RSTR |= RCC_AHB2RSTR_DAC4RST; __DSB();
			RCC->AHB2RSTR &= ~RCC_AHB2RSTR_DAC4RST;
		}
	if constexpr (peripheral == Peripheral::Dma1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; __DSB();
			RCC->AHB1RSTR |= RCC_AHB1RSTR_DMA1RST; __DSB();
			RCC->AHB1RSTR &= ~RCC_AHB1RSTR_DMA1RST;
		}
	if constexpr (peripheral == Peripheral::Dma2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; __DSB();
			RCC->AHB1RSTR |= RCC_AHB1RSTR_DMA2RST; __DSB();
			RCC->AHB1RSTR &= ~RCC_AHB1RSTR_DMA2RST;
		}
	if constexpr (peripheral == Peripheral::Dmamux1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN; __DSB();
			RCC->AHB1RSTR |= RCC_AHB1RSTR_DMAMUX1RST; __DSB();
			RCC->AHB1RSTR &= ~RCC_AHB1RSTR_DMAMUX1RST;
		}
	if constexpr (peripheral == Peripheral::Fdcan1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_FDCANRST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_FDCANRST;
		}
	if constexpr (peripheral == Peripheral::Flash)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB1ENR |= RCC_AHB1ENR_FLASHEN; __DSB();
			RCC->AHB1RSTR |= RCC_AHB1RSTR_FLASHRST; __DSB();
			RCC->AHB1RSTR &= ~RCC_AHB1RSTR_FLASHRST;
		}
	if constexpr (peripheral == Peripheral::Fmac)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB1ENR |= RCC_AHB1ENR_FMACEN; __DSB();
			RCC->AHB1RSTR |= RCC_AHB1RSTR_FMACRST; __DSB();
			RCC->AHB1RSTR &= ~RCC_AHB1RSTR_FMACRST;
		}
	if constexpr (peripheral == Peripheral::Hrtim1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_HRTIM1EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_HRTIM1RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_HRTIM1RST;
		}
	if constexpr (peripheral == Peripheral::I2c1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C1RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C1RST;
		}
	if constexpr (peripheral == Peripheral::I2c2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C2RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C2RST;
		}
	if constexpr (peripheral == Peripheral::I2c3)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_I2C3EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C3RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C3RST;
		}
	if constexpr (peripheral == Peripheral::I2c4)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR2 |= RCC_APB1ENR2_I2C4EN; __DSB();
			RCC->APB1RSTR2 |= RCC_APB1RSTR2_I2C4RST; __DSB();
			RCC->APB1RSTR2 &= ~RCC_APB1RSTR2_I2C4RST;
		}
	if constexpr (peripheral == Peripheral::Lptim1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_LPTIM1RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_LPTIM1RST;
		}
	if constexpr (peripheral == Peripheral::Lpuart1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN; __DSB();
			RCC->APB1RSTR2 |= RCC_APB1RSTR2_LPUART1RST; __DSB();
			RCC->APB1RSTR2 &= ~RCC_APB1RSTR2_LPUART1RST;
		}
	if constexpr (peripheral == Peripheral::Rng)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN; __DSB();
			RCC->AHB2RSTR |= RCC_AHB2RSTR_RNGRST; __DSB();
			RCC->AHB2RSTR &= ~RCC_AHB2RSTR_RNGRST;
		}
	if constexpr (peripheral == Peripheral::Rtc)
		{
			RCC->APB1ENR1 |= RCC_APB1ENR1_RTCAPBEN;
		}
	if constexpr (peripheral == Peripheral::Sai1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_SAI1EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_SAI1RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_SAI1RST;
		}
	if constexpr (peripheral == Peripheral::Spi1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
		}
	if constexpr (peripheral == Peripheral::Spi2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_SPI2RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_SPI2RST;
		}
	if constexpr (peripheral == Peripheral::Spi3)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_SPI3EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_SPI3RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_SPI3RST;
		}
	if constexpr (peripheral == Peripheral::Tim1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST;
		}
	if constexpr (peripheral == Peripheral::Tim15)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM15RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM15RST;
		}
	if constexpr (peripheral == Peripheral::Tim16)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM16EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM16RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM16RST;
		}
	if constexpr (peripheral == Peripheral::Tim17)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM17EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM17RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM17RST;
		}
	if constexpr (peripheral == Peripheral::Tim2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM2RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM2RST;
		}
	if constexpr (peripheral == Peripheral::Tim20)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM20EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM20RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM20RST;
		}
	if constexpr (peripheral == Peripheral::Tim3)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM3RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM3RST;
		}
	if constexpr (peripheral == Peripheral::Tim4)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM4RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM4RST;
		}
	if constexpr (peripheral == Peripheral::Tim5)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM5RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM5RST;
		}
	if constexpr (peripheral == Peripheral::Tim6)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM6RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM6RST;
		}
	if constexpr (peripheral == Peripheral::Tim7)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM7RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM7RST;
		}
	if constexpr (peripheral == Peripheral::Tim8)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM8EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM8RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM8RST;
		}
	if constexpr (peripheral == Peripheral::Ucpd1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR2 |= RCC_APB1ENR2_UCPD1EN; __DSB();
			RCC->APB1RSTR2 |= RCC_APB1RSTR2_UCPD1RST; __DSB();
			RCC->APB1RSTR2 &= ~RCC_APB1RSTR2_UCPD1RST;
		}
	if constexpr (peripheral == Peripheral::Usart1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_USART1EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
		}
	if constexpr (peripheral == Peripheral::Usart2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_USART2RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_USART2RST;
		}
	if constexpr (peripheral == Peripheral::Usart3)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_USART3RST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_USART3RST;
		}
	if constexpr (peripheral == Peripheral::Usb)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR1 |= RCC_APB1ENR1_USBEN; __DSB();
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_USBRST; __DSB();
			RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_USBRST;
		}
	if constexpr (peripheral == Peripheral::Wwdg)
		{
			RCC->APB1ENR1 |= RCC_APB1ENR1_WWDGEN;
		}
	__DSB();
}

template< Peripheral peripheral >
void
Rcc::disable()
{
	static_assert(rcc_check_enable(peripheral),
		"Rcc::disable() doesn't know this peripheral!");

	__DSB();
	if constexpr (peripheral == Peripheral::Cordic) {
		RCC->AHB1ENR &= ~RCC_AHB1ENR_CORDICEN;
	}
	if constexpr (peripheral == Peripheral::Crc) {
		RCC->AHB1ENR &= ~RCC_AHB1ENR_CRCEN;
	}
	if constexpr (peripheral == Peripheral::Dac1) {
		RCC->AHB2ENR &= ~RCC_AHB2ENR_DAC1EN;
	}
	if constexpr (peripheral == Peripheral::Dac2) {
		RCC->AHB2ENR &= ~RCC_AHB2ENR_DAC2EN;
	}
	if constexpr (peripheral == Peripheral::Dac3) {
		RCC->AHB2ENR &= ~RCC_AHB2ENR_DAC3EN;
	}
	if constexpr (peripheral == Peripheral::Dac4) {
		RCC->AHB2ENR &= ~RCC_AHB2ENR_DAC4EN;
	}
	if constexpr (peripheral == Peripheral::Dma1) {
		RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA1EN;
	}
	if constexpr (peripheral == Peripheral::Dma2) {
		RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA2EN;
	}
	if constexpr (peripheral == Peripheral::Dmamux1) {
		RCC->AHB1ENR &= ~RCC_AHB1ENR_DMAMUX1EN;
	}
	if constexpr (peripheral == Peripheral::Fdcan1) {
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_FDCANEN;
	}
	if constexpr (peripheral == Peripheral::Flash) {
		RCC->AHB1ENR &= ~RCC_AHB1ENR_FLASHEN;
	}
	if constexpr (peripheral == Peripheral::Fmac) {
		RCC->AHB1ENR &= ~RCC_AHB1ENR_FMACEN;
	}
	if constexpr (peripheral == Peripheral::Hrtim1) {
		RCC->APB2ENR &= ~RCC_APB2ENR_HRTIM1EN;
	}
	if constexpr (peripheral == Peripheral::I2c1) {
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_I2C1EN;
	}
	if constexpr (peripheral == Peripheral::I2c2) {
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_I2C2EN;
	}
	if constexpr (peripheral == Peripheral::I2c3) {
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_I2C3EN;
	}
	if constexpr (peripheral == Peripheral::I2c4) {
		RCC->APB1ENR2 &= ~RCC_APB1ENR2_I2C4EN;
	}
	if constexpr (peripheral == Peripheral::Lptim1) {
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_LPTIM1EN;
	}
	if constexpr (peripheral == Peripheral::Lpuart1) {
		RCC->APB1ENR2 &= ~RCC_APB1ENR2_LPUART1EN;
	}
	if constexpr (peripheral == Peripheral::Rng) {
		RCC->AHB2ENR &= ~RCC_AHB2ENR_RNGEN;
	}
	if constexpr (peripheral == Peripheral::Rtc) {
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_RTCAPBEN;
	}
	if constexpr (peripheral == Peripheral::Sai1) {
		RCC->APB2ENR &= ~RCC_APB2ENR_SAI1EN;
	}
	if constexpr (peripheral == Peripheral::Spi1) {
		RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
	}
	if constexpr (peripheral == Peripheral::Spi2) {
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_SPI2EN;
	}
	if constexpr (peripheral == Peripheral::Spi3) {
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_SPI3EN;
	}
	if constexpr (peripheral == Peripheral::Tim1) {
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN;
	}
	if constexpr (peripheral == Peripheral::Tim15) {
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM15EN;
	}
	if constexpr (peripheral == Peripheral::Tim16) {
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM16EN;
	}
	if constexpr (peripheral == Peripheral::Tim17) {
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM17EN;
	}
	if constexpr (peripheral == Peripheral::Tim2) {
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM2EN;
	}
	if constexpr (peripheral == Peripheral::Tim20) {
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM20EN;
	}
	if constexpr (peripheral == Peripheral::Tim3) {
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM3EN;
	}
	if constexpr (peripheral == Peripheral::Tim4) {
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM4EN;
	}
	if constexpr (peripheral == Peripheral::Tim5) {
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM5EN;
	}
	if constexpr (peripheral == Peripheral::Tim6) {
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM6EN;
	}
	if constexpr (peripheral == Peripheral::Tim7) {
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM7EN;
	}
	if constexpr (peripheral == Peripheral::Tim8) {
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM8EN;
	}
	if constexpr (peripheral == Peripheral::Ucpd1) {
		RCC->APB1ENR2 &= ~RCC_APB1ENR2_UCPD1EN;
	}
	if constexpr (peripheral == Peripheral::Usart1) {
		RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
	}
	if constexpr (peripheral == Peripheral::Usart2) {
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_USART2EN;
	}
	if constexpr (peripheral == Peripheral::Usart3) {
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_USART3EN;
	}
	if constexpr (peripheral == Peripheral::Usb) {
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_USBEN;
	}
	if constexpr (peripheral == Peripheral::Wwdg) {
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_WWDGEN;
	}
	__DSB();
}

template< Peripheral peripheral >
bool
Rcc::isEnabled()
{
	static_assert(rcc_check_enable(peripheral),
		"Rcc::isEnabled() doesn't know this peripheral!");

	if constexpr (peripheral == Peripheral::Cordic)
		return RCC->AHB1ENR & RCC_AHB1ENR_CORDICEN;
	if constexpr (peripheral == Peripheral::Crc)
		return RCC->AHB1ENR & RCC_AHB1ENR_CRCEN;
	if constexpr (peripheral == Peripheral::Dac1)
		return RCC->AHB2ENR & RCC_AHB2ENR_DAC1EN;
	if constexpr (peripheral == Peripheral::Dac2)
		return RCC->AHB2ENR & RCC_AHB2ENR_DAC2EN;
	if constexpr (peripheral == Peripheral::Dac3)
		return RCC->AHB2ENR & RCC_AHB2ENR_DAC3EN;
	if constexpr (peripheral == Peripheral::Dac4)
		return RCC->AHB2ENR & RCC_AHB2ENR_DAC4EN;
	if constexpr (peripheral == Peripheral::Dma1)
		return RCC->AHB1ENR & RCC_AHB1ENR_DMA1EN;
	if constexpr (peripheral == Peripheral::Dma2)
		return RCC->AHB1ENR & RCC_AHB1ENR_DMA2EN;
	if constexpr (peripheral == Peripheral::Dmamux1)
		return RCC->AHB1ENR & RCC_AHB1ENR_DMAMUX1EN;
	if constexpr (peripheral == Peripheral::Fdcan1)
		return RCC->APB1ENR1 & RCC_APB1ENR1_FDCANEN;
	if constexpr (peripheral == Peripheral::Flash)
		return RCC->AHB1ENR & RCC_AHB1ENR_FLASHEN;
	if constexpr (peripheral == Peripheral::Fmac)
		return RCC->AHB1ENR & RCC_AHB1ENR_FMACEN;
	if constexpr (peripheral == Peripheral::Hrtim1)
		return RCC->APB2ENR & RCC_APB2ENR_HRTIM1EN;
	if constexpr (peripheral == Peripheral::I2c1)
		return RCC->APB1ENR1 & RCC_APB1ENR1_I2C1EN;
	if constexpr (peripheral == Peripheral::I2c2)
		return RCC->APB1ENR1 & RCC_APB1ENR1_I2C2EN;
	if constexpr (peripheral == Peripheral::I2c3)
		return RCC->APB1ENR1 & RCC_APB1ENR1_I2C3EN;
	if constexpr (peripheral == Peripheral::I2c4)
		return RCC->APB1ENR2 & RCC_APB1ENR2_I2C4EN;
	if constexpr (peripheral == Peripheral::Lptim1)
		return RCC->APB1ENR1 & RCC_APB1ENR1_LPTIM1EN;
	if constexpr (peripheral == Peripheral::Lpuart1)
		return RCC->APB1ENR2 & RCC_APB1ENR2_LPUART1EN;
	if constexpr (peripheral == Peripheral::Rng)
		return RCC->AHB2ENR & RCC_AHB2ENR_RNGEN;
	if constexpr (peripheral == Peripheral::Rtc)
		return RCC->APB1ENR1 & RCC_APB1ENR1_RTCAPBEN;
	if constexpr (peripheral == Peripheral::Sai1)
		return RCC->APB2ENR & RCC_APB2ENR_SAI1EN;
	if constexpr (peripheral == Peripheral::Spi1)
		return RCC->APB2ENR & RCC_APB2ENR_SPI1EN;
	if constexpr (peripheral == Peripheral::Spi2)
		return RCC->APB1ENR1 & RCC_APB1ENR1_SPI2EN;
	if constexpr (peripheral == Peripheral::Spi3)
		return RCC->APB1ENR1 & RCC_APB1ENR1_SPI3EN;
	if constexpr (peripheral == Peripheral::Tim1)
		return RCC->APB2ENR & RCC_APB2ENR_TIM1EN;
	if constexpr (peripheral == Peripheral::Tim15)
		return RCC->APB2ENR & RCC_APB2ENR_TIM15EN;
	if constexpr (peripheral == Peripheral::Tim16)
		return RCC->APB2ENR & RCC_APB2ENR_TIM16EN;
	if constexpr (peripheral == Peripheral::Tim17)
		return RCC->APB2ENR & RCC_APB2ENR_TIM17EN;
	if constexpr (peripheral == Peripheral::Tim2)
		return RCC->APB1ENR1 & RCC_APB1ENR1_TIM2EN;
	if constexpr (peripheral == Peripheral::Tim20)
		return RCC->APB2ENR & RCC_APB2ENR_TIM20EN;
	if constexpr (peripheral == Peripheral::Tim3)
		return RCC->APB1ENR1 & RCC_APB1ENR1_TIM3EN;
	if constexpr (peripheral == Peripheral::Tim4)
		return RCC->APB1ENR1 & RCC_APB1ENR1_TIM4EN;
	if constexpr (peripheral == Peripheral::Tim5)
		return RCC->APB1ENR1 & RCC_APB1ENR1_TIM5EN;
	if constexpr (peripheral == Peripheral::Tim6)
		return RCC->APB1ENR1 & RCC_APB1ENR1_TIM6EN;
	if constexpr (peripheral == Peripheral::Tim7)
		return RCC->APB1ENR1 & RCC_APB1ENR1_TIM7EN;
	if constexpr (peripheral == Peripheral::Tim8)
		return RCC->APB2ENR & RCC_APB2ENR_TIM8EN;
	if constexpr (peripheral == Peripheral::Ucpd1)
		return RCC->APB1ENR2 & RCC_APB1ENR2_UCPD1EN;
	if constexpr (peripheral == Peripheral::Usart1)
		return RCC->APB2ENR & RCC_APB2ENR_USART1EN;
	if constexpr (peripheral == Peripheral::Usart2)
		return RCC->APB1ENR1 & RCC_APB1ENR1_USART2EN;
	if constexpr (peripheral == Peripheral::Usart3)
		return RCC->APB1ENR1 & RCC_APB1ENR1_USART3EN;
	if constexpr (peripheral == Peripheral::Usb)
		return RCC->APB1ENR1 & RCC_APB1ENR1_USBEN;
	if constexpr (peripheral == Peripheral::Wwdg)
		return RCC->APB1ENR1 & RCC_APB1ENR1_WWDGEN;
}

}   // namespace modm::platform
