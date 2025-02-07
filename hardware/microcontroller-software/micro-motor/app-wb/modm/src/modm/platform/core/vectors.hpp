/*
 * Copyright (c) 2021, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

#include <stdint.h>
#include <modm/architecture/utils.hpp>
#include <string_view>

extern "C"
{

void Reset_Handler(void);
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void WWDG_IRQHandler(void);
void PVD_PVM_IRQHandler(void);
void RTC_TAMP_LSECSS_IRQHandler(void);
void RTC_WKUP_IRQHandler(void);
void FLASH_IRQHandler(void);
void RCC_IRQHandler(void);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void DMA1_Channel1_IRQHandler(void);
void DMA1_Channel2_IRQHandler(void);
void DMA1_Channel3_IRQHandler(void);
void DMA1_Channel4_IRQHandler(void);
void DMA1_Channel5_IRQHandler(void);
void DMA1_Channel6_IRQHandler(void);
void DMA1_Channel7_IRQHandler(void);
void ADC1_2_IRQHandler(void);
void USB_HP_IRQHandler(void);
void USB_LP_IRQHandler(void);
void FDCAN1_IT0_IRQHandler(void);
void FDCAN1_IT1_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void TIM1_BRK_TIM15_IRQHandler(void);
void TIM1_UP_TIM16_IRQHandler(void);
void TIM1_TRG_COM_TIM17_IRQHandler(void);
void TIM1_CC_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
void I2C2_EV_IRQHandler(void);
void I2C2_ER_IRQHandler(void);
void SPI1_IRQHandler(void);
void SPI2_IRQHandler(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void RTC_Alarm_IRQHandler(void);
void USBWakeUp_IRQHandler(void);
void TIM8_BRK_IRQHandler(void);
void TIM8_UP_IRQHandler(void);
void TIM8_TRG_COM_IRQHandler(void);
void TIM8_CC_IRQHandler(void);
void ADC3_IRQHandler(void);
void FMC_IRQHandler(void);
void LPTIM1_IRQHandler(void);
void TIM5_IRQHandler(void);
void SPI3_IRQHandler(void);
void UART4_IRQHandler(void);
void UART5_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);
void TIM7_DAC_IRQHandler(void);
void DMA2_Channel1_IRQHandler(void);
void DMA2_Channel2_IRQHandler(void);
void DMA2_Channel3_IRQHandler(void);
void DMA2_Channel4_IRQHandler(void);
void DMA2_Channel5_IRQHandler(void);
void ADC4_IRQHandler(void);
void ADC5_IRQHandler(void);
void UCPD1_IRQHandler(void);
void COMP1_2_3_IRQHandler(void);
void COMP4_5_6_IRQHandler(void);
void COMP7_IRQHandler(void);
void HRTIM1_Master_IRQHandler(void);
void HRTIM1_TIMA_IRQHandler(void);
void HRTIM1_TIMB_IRQHandler(void);
void HRTIM1_TIMC_IRQHandler(void);
void HRTIM1_TIMD_IRQHandler(void);
void HRTIM1_TIME_IRQHandler(void);
void HRTIM1_FLT_IRQHandler(void);
void HRTIM1_TIMF_IRQHandler(void);
void CRS_IRQHandler(void);
void SAI1_IRQHandler(void);
void TIM20_BRK_IRQHandler(void);
void TIM20_UP_IRQHandler(void);
void TIM20_TRG_COM_IRQHandler(void);
void TIM20_CC_IRQHandler(void);
void FPU_IRQHandler(void);
void I2C4_EV_IRQHandler(void);
void I2C4_ER_IRQHandler(void);
void SPI4_IRQHandler(void);
void FDCAN2_IT0_IRQHandler(void);
void FDCAN2_IT1_IRQHandler(void);
void FDCAN3_IT0_IRQHandler(void);
void FDCAN3_IT1_IRQHandler(void);
void RNG_IRQHandler(void);
void LPUART1_IRQHandler(void);
void I2C3_EV_IRQHandler(void);
void I2C3_ER_IRQHandler(void);
void DMAMUX_OVR_IRQHandler(void);
void QUADSPI_IRQHandler(void);
void DMA1_Channel8_IRQHandler(void);
void DMA2_Channel6_IRQHandler(void);
void DMA2_Channel7_IRQHandler(void);
void DMA2_Channel8_IRQHandler(void);
void CORDIC_IRQHandler(void);
void FMAC_IRQHandler(void);
}

namespace modm::platform::detail
{

constexpr std::string_view vectorNames[] =
{
	"__main_stack_top",
	"Reset",
	"NMI",
	"HardFault",
	"MemManage",
	"BusFault",
	"UsageFault",
	"Undefined",
	"Undefined",
	"Undefined",
	"Undefined",
	"SVC",
	"DebugMon",
	"Undefined",
	"PendSV",
	"SysTick",
	"WWDG",
	"PVD_PVM",
	"RTC_TAMP_LSECSS",
	"RTC_WKUP",
	"FLASH",
	"RCC",
	"EXTI0",
	"EXTI1",
	"EXTI2",
	"EXTI3",
	"EXTI4",
	"DMA1_Channel1",
	"DMA1_Channel2",
	"DMA1_Channel3",
	"DMA1_Channel4",
	"DMA1_Channel5",
	"DMA1_Channel6",
	"DMA1_Channel7",
	"ADC1_2",
	"USB_HP",
	"USB_LP",
	"FDCAN1_IT0",
	"FDCAN1_IT1",
	"EXTI9_5",
	"TIM1_BRK_TIM15",
	"TIM1_UP_TIM16",
	"TIM1_TRG_COM_TIM17",
	"TIM1_CC",
	"TIM2",
	"TIM3",
	"TIM4",
	"I2C1_EV",
	"I2C1_ER",
	"I2C2_EV",
	"I2C2_ER",
	"SPI1",
	"SPI2",
	"USART1",
	"USART2",
	"USART3",
	"EXTI15_10",
	"RTC_Alarm",
	"USBWakeUp",
	"TIM8_BRK",
	"TIM8_UP",
	"TIM8_TRG_COM",
	"TIM8_CC",
	"ADC3",
	"FMC",
	"LPTIM1",
	"TIM5",
	"SPI3",
	"UART4",
	"UART5",
	"TIM6_DAC",
	"TIM7_DAC",
	"DMA2_Channel1",
	"DMA2_Channel2",
	"DMA2_Channel3",
	"DMA2_Channel4",
	"DMA2_Channel5",
	"ADC4",
	"ADC5",
	"UCPD1",
	"COMP1_2_3",
	"COMP4_5_6",
	"COMP7",
	"HRTIM1_Master",
	"HRTIM1_TIMA",
	"HRTIM1_TIMB",
	"HRTIM1_TIMC",
	"HRTIM1_TIMD",
	"HRTIM1_TIME",
	"HRTIM1_FLT",
	"HRTIM1_TIMF",
	"CRS",
	"SAI1",
	"TIM20_BRK",
	"TIM20_UP",
	"TIM20_TRG_COM",
	"TIM20_CC",
	"FPU",
	"I2C4_EV",
	"I2C4_ER",
	"SPI4",
	"Undefined",
	"FDCAN2_IT0",
	"FDCAN2_IT1",
	"FDCAN3_IT0",
	"FDCAN3_IT1",
	"RNG",
	"LPUART1",
	"I2C3_EV",
	"I2C3_ER",
	"DMAMUX_OVR",
	"QUADSPI",
	"DMA1_Channel8",
	"DMA2_Channel6",
	"DMA2_Channel7",
	"DMA2_Channel8",
	"CORDIC",
	"FMAC",
};


#ifndef MODM_ISR_DISABLE_VALIDATION
#define MODM_ISR_VALIDATE(vector_str, vector) \
	static_assert(::modm::platform::detail::validateIrqName(vector_str), \
			"'" vector_str "' is not a valid IRQ name!\n" \
			"  Hint: You do not need to add '_IRQHandler' to the name.\n" \
			"  Hint: Here are all the IRQs on this device:\n" \
			"    - WWDG\n" \
			"    - PVD_PVM\n" \
			"    - RTC_TAMP_LSECSS\n" \
			"    - RTC_WKUP\n" \
			"    - FLASH\n" \
			"    - RCC\n" \
			"    - EXTI0\n" \
			"    - EXTI1\n" \
			"    - EXTI2\n" \
			"    - EXTI3\n" \
			"    - EXTI4\n" \
			"    - DMA1_Channel1\n" \
			"    - DMA1_Channel2\n" \
			"    - DMA1_Channel3\n" \
			"    - DMA1_Channel4\n" \
			"    - DMA1_Channel5\n" \
			"    - DMA1_Channel6\n" \
			"    - DMA1_Channel7\n" \
			"    - ADC1_2\n" \
			"    - USB_HP\n" \
			"    - USB_LP\n" \
			"    - FDCAN1_IT0\n" \
			"    - FDCAN1_IT1\n" \
			"    - EXTI9_5\n" \
			"    - TIM1_BRK_TIM15\n" \
			"    - TIM1_UP_TIM16\n" \
			"    - TIM1_TRG_COM_TIM17\n" \
			"    - TIM1_CC\n" \
			"    - TIM2\n" \
			"    - TIM3\n" \
			"    - TIM4\n" \
			"    - I2C1_EV\n" \
			"    - I2C1_ER\n" \
			"    - I2C2_EV\n" \
			"    - I2C2_ER\n" \
			"    - SPI1\n" \
			"    - SPI2\n" \
			"    - USART1\n" \
			"    - USART2\n" \
			"    - USART3\n" \
			"    - EXTI15_10\n" \
			"    - RTC_Alarm\n" \
			"    - USBWakeUp\n" \
			"    - TIM8_BRK\n" \
			"    - TIM8_UP\n" \
			"    - TIM8_TRG_COM\n" \
			"    - TIM8_CC\n" \
			"    - ADC3\n" \
			"    - FMC\n" \
			"    - LPTIM1\n" \
			"    - TIM5\n" \
			"    - SPI3\n" \
			"    - UART4\n" \
			"    - UART5\n" \
			"    - TIM6_DAC\n" \
			"    - TIM7_DAC\n" \
			"    - DMA2_Channel1\n" \
			"    - DMA2_Channel2\n" \
			"    - DMA2_Channel3\n" \
			"    - DMA2_Channel4\n" \
			"    - DMA2_Channel5\n" \
			"    - ADC4\n" \
			"    - ADC5\n" \
			"    - UCPD1\n" \
			"    - COMP1_2_3\n" \
			"    - COMP4_5_6\n" \
			"    - COMP7\n" \
			"    - HRTIM1_Master\n" \
			"    - HRTIM1_TIMA\n" \
			"    - HRTIM1_TIMB\n" \
			"    - HRTIM1_TIMC\n" \
			"    - HRTIM1_TIMD\n" \
			"    - HRTIM1_TIME\n" \
			"    - HRTIM1_FLT\n" \
			"    - HRTIM1_TIMF\n" \
			"    - CRS\n" \
			"    - SAI1\n" \
			"    - TIM20_BRK\n" \
			"    - TIM20_UP\n" \
			"    - TIM20_TRG_COM\n" \
			"    - TIM20_CC\n" \
			"    - FPU\n" \
			"    - I2C4_EV\n" \
			"    - I2C4_ER\n" \
			"    - SPI4\n" \
			"    - FDCAN2_IT0\n" \
			"    - FDCAN2_IT1\n" \
			"    - FDCAN3_IT0\n" \
			"    - FDCAN3_IT1\n" \
			"    - RNG\n" \
			"    - LPUART1\n" \
			"    - I2C3_EV\n" \
			"    - I2C3_ER\n" \
			"    - DMAMUX_OVR\n" \
			"    - QUADSPI\n" \
			"    - DMA1_Channel8\n" \
			"    - DMA2_Channel6\n" \
			"    - DMA2_Channel7\n" \
			"    - DMA2_Channel8\n" \
			"    - CORDIC\n" \
			"    - FMAC\n" \
	)
#else
#define MODM_ISR_VALIDATE(...)
#endif

constexpr int getIrqPosition(std::string_view name)
{
	for (int pos = 0; pos < 118; pos++)
		if (vectorNames[pos] == name) return pos;
	return -1;
}

constexpr bool validateIrqName(std::string_view name)
{
	return getIrqPosition(name) != -1;
}

}	// namespace modm::platform::detail
