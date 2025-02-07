/* encoder_interrupt.hpp
*
* Copyright (C) 2024 Henrik Hose
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ENCODER_INTERRUPT_HPP
#define ENCODER_INTERRUPT_HPP

#include <micro-motor/hardware.hpp>
#include <modm/platform.hpp>
#include <modm/processing/timer.hpp>
#include <micro-motor/hardware.hpp>

#include "../data.hpp"
#include "../encoder.hpp"

// all these are only used once with this encoder...
static uint16_t outtrig{0x7FFE};
static uint16_t outzero{0};
static uint16_t inbuf{0};
static uint16_t inbuf2{0};


MODM_ISR(TIM8_CC)
{
    if (Timer8::getInterruptFlags() & Timer8::InterruptFlag::CaptureCompare1){
        Timer8::acknowledgeInterruptFlags(Timer8::InterruptFlag::CaptureCompare1);
        Board::Encoder::As5047::Cs::reset();
        SpiHal1::write(outtrig);
        return;
    }

    if (Timer8::getInterruptFlags() & Timer8::InterruptFlag::CaptureCompare2){
        Timer8::acknowledgeInterruptFlags(Timer8::InterruptFlag::CaptureCompare2);
        SpiHal1::read(inbuf2);
        Board::Encoder::As5047::Cs::set();
        return;
    }

    if (Timer8::getInterruptFlags() & Timer8::InterruptFlag::CaptureCompare3){
        Timer8::acknowledgeInterruptFlags(Timer8::InterruptFlag::CaptureCompare3);
        Board::Encoder::As5047::Cs::reset();
        SpiHal1::write(outzero);
        return;
    }

    if (Timer8::getInterruptFlags() & Timer8::InterruptFlag::CaptureCompare4){
        Timer8::acknowledgeInterruptFlags(Timer8::InterruptFlag::CaptureCompare4);
        SpiHal1::read(inbuf);
        Board::Encoder::As5047::Cs::set();
        updateEncoder(inbuf);
        return;
    }
}

#endif // ENCODER_INTERRUPT_HPP
