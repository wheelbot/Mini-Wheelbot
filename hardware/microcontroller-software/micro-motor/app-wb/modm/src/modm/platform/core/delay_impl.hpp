/*
 * Copyright (c) 2009, Martin Rosekeit
 * Copyright (c) 2009-2011, Fabian Greif
 * Copyright (c) 2010, Georgi Grinshpun
 * Copyright (c) 2012, 2014, Sascha Schade
 * Copyright (c) 2012, 2014-2016, 2019-2021 Niklas Hauser
 * Copyright (c) 2014, Kevin Läufer
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once
#include <cmath>
#include <chrono>
#include "delay_ns.hpp"
#include <modm/architecture/interface/assert.hpp>
/// @cond
#define MODM_DELAY_NS_IS_ACCURATE 1

namespace modm
{
namespace platform
{
extern uint16_t delay_ns_per_loop;
extern uint16_t delay_fcpu_MHz;

constexpr uint8_t delay_fcpu_MHz_shift(4);
constexpr uint16_t computeDelayMhz(uint32_t hz)
{ return std::round(hz / 1'000'000.f * (1ul << delay_fcpu_MHz_shift)); }
}

modm_always_inline
void delay_ns(uint32_t ns)
{
    asm volatile(
        "mov r0, %0 \n\t"
        "blx %1"
        :: "r" (ns), "l" (platform::delay_ns) : "r0", "r1", "r2", "lr");
}


void delay_us(uint32_t us);
inline void delay_ms(uint32_t ms)
{
    while(1)
    {
        if (ms <= 1000) {
            delay_us(ms * 1000);
            break;
        }
        delay_us(1'000'000);
        ms -= 1000;
    }
}

}
/// @endcond
