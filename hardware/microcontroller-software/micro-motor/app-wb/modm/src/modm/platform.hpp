/*
 * Copyright (c) 2016, Sascha Schade
 * Copyright (c) 2017, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include <modm/architecture.hpp>

#include "platform/adc/adc_1.hpp"
#include "platform/adc/adc_2.hpp"
#include "platform/adc/adc_3.hpp"
#include "platform/adc/adc_4.hpp"
#include "platform/adc/adc_interrupt_1.hpp"
#include "platform/adc/adc_interrupt_2.hpp"
#include "platform/adc/adc_interrupt_3.hpp"
#include "platform/adc/adc_interrupt_4.hpp"
#include "platform/can/can_1.hpp"
#include "platform/can/can_bit_timings.hpp"
#include "platform/can/can_message_ram_config.hpp"
#include "platform/can/message_ram.hpp"
#include "platform/clock/rcc.hpp"
#include "platform/clock/systick_timer.hpp"
#include "platform/comp/base.hpp"
#include "platform/comp/comp_1.hpp"
#include "platform/comp/comp_2.hpp"
#include "platform/comp/comp_3.hpp"
#include "platform/core/delay_ns.hpp"
#include "platform/core/hardware_init.hpp"
#include "platform/core/heap_table.hpp"
#include "platform/core/vectors.hpp"
#include "platform/dac/dac_1.hpp"
#include "platform/dac/dac_2.hpp"
#include "platform/flash/flash.hpp"
#include "platform/gpio/base.hpp"
#include "platform/gpio/connector.hpp"
#include "platform/gpio/data.hpp"
#include "platform/gpio/inverted.hpp"
#include "platform/gpio/pins.hpp"
#include "platform/gpio/port.hpp"
#include "platform/gpio/port_shim.hpp"
#include "platform/gpio/set.hpp"
#include "platform/gpio/software_port.hpp"
#include "platform/gpio/static.hpp"
#include "platform/gpio/unused.hpp"
#include "platform/spi/bitbang_spi_master.hpp"
#include "platform/spi/spi_base.hpp"
#include "platform/spi/spi_hal_1.hpp"
#include "platform/spi/spi_master_1.hpp"
#include "platform/spi/uart_spi_master_3.hpp"
#include "platform/timer/advanced_base.hpp"
#include "platform/timer/basic_base.hpp"
#include "platform/timer/general_purpose_base.hpp"
#include "platform/timer/timer_1.hpp"
#include "platform/timer/timer_3.hpp"
#include "platform/timer/timer_8.hpp"
#include "platform/uart/uart.hpp"
#include "platform/uart/uart_base.hpp"
#include "platform/uart/uart_buffer.hpp"
#include "platform/uart/uart_hal_1.hpp"
#include "platform/uart/uart_hal_3.hpp"
