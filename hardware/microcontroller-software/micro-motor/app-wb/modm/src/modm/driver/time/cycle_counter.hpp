/*
 * Copyright (c) 2023, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once
#include <cstdint>
#include <modm/platform.hpp>

namespace modm
{

/// @ingroup modm_driver_cycle_counter
class CycleCounter
{
public:
    using cycle_t = uint32_t;
    /// @param force overwrite previous timer configuration
    /// @param overhead total number of overhead cycles
    void
    initialize(bool force [[maybe_unused]] = false, uint8_t overhead = 4)
    {
        this->overhead = overhead;
    }

    /// Sample the timer at the start of your measurement
    modm_always_inline void
    start()
    {
        start_ = DWT->CYCCNT;
    }

    /// Sample the timer at the end of your measurement
    modm_always_inline void
    stop()
    {
        stop_ = DWT->CYCCNT;
    }

    /// @return the difference in cycles between start and stop.
    cycle_t
    cycles()
    {
        return (stop_ - start_) - overhead;
    }

    /// @return the difference in milliseconds between start and stop.
    uint32_t
    milliseconds()
    {
        return (cycles() * 1'000ull) / SystemCoreClock;
    }

    /// @return the difference in microseconds between start and stop.
    uint32_t
    microseconds()
    {
        return (cycles() * 1'000'000ull) / SystemCoreClock;
    }

    /// @return the difference in nanoseconds between start and stop.
    uint32_t
    nanoseconds()
    {
        return (cycles() * 1'000'000'000ull) / SystemCoreClock;
    }

private:
    cycle_t start_, stop_;
    uint8_t overhead{4};
};

}
