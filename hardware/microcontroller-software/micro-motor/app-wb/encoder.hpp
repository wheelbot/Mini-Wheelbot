/* encoder.hpp
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

#ifndef ENCODER_HPP
#define ENCODER_HPP

#include "data.hpp"

inline
float
raw_encoder_to_electric_aligned_degrees(const Configuration& config, const int32_t& raw_value){
    if (config.persistent_data.inverted)
    {
        return -float{
                (raw_value - config.persistent_data.phase_offset)
            }* config.encoder.degrees_per_tick;
    }
    else
    {
        return float{
                (raw_value -  config.persistent_data.phase_offset )
            } * config.encoder.degrees_per_tick;
    }
}

inline
void updateEncoder(const uint16_t raw_candidate){

    Configuration::Encoder& enc = main_configuration.encoder;

    // check parity bit, this is AS5047 specific
    if ( !checkMsbEvenParity(raw_candidate) ){
        main_configuration.encoder.corrupted_data_counter += 1;
        return; // nothing to do, data is corrupt
    }
    if (enc.corrupted_data_counter > 0) enc.corrupted_data_counter-=1;

    // check 0xffff or 0x000
    if ( ( raw_candidate == 0x0000 ) || ( raw_candidate == 0xffff )
        // || ( (raw_candidate & 0x3fff) == 0x0000 ) || ( (raw_candidate & 0x3fff) == 0xffff )
    ){
        enc.not_connected_counter += 1;
        return; // nothing to do, data is corrupt
    }
    if (enc.not_connected_counter > 0) enc.not_connected_counter-=1;


    // if reset was requested
    if (enc.reset){
        enc.reset = false;

        // reset calibrated first to avoid race condition
        // enc.calibrated = false;
        enc.corrupted_data_counter = 0; // increments every time corrupted or no data is read

        enc.raw_value.store(static_cast<int16_t>(raw_candidate & 0x3fff), std::memory_order_relaxed);
        enc.position.store(enc.raw_value, std::memory_order_relaxed);
        enc.last_raw_value.store(enc.raw_value, std::memory_order_relaxed);
        // enc.diff = 0;
        // MODM_LOG_DEBUG << "Encoder was reset!" << modm::endl;
    }

    // data is fine so we can get the value
    enc.raw_value = static_cast<int16_t>(raw_candidate & 0x3fff);


    int32_t diff = enc.raw_value - enc.last_raw_value;
    if (diff > enc.resolution/2) { // Overflow handling
        diff -= enc.resolution;
    } else if (diff < -enc.resolution/2) { // Underflow handling
        diff += enc.resolution;
    }
    enc.position += diff;
    enc.last_raw_value.store(enc.raw_value.load(std::memory_order_relaxed), std::memory_order_relaxed);
    if (main_configuration.persistent_data.inverted)
    {
        enc.angle_degrees_aligned_with_electrical_frame =
            -float{
                (enc.raw_value- main_configuration.persistent_data.phase_offset)
            }* enc.degrees_per_tick;
    }
    else
    {
        enc.angle_degrees_aligned_with_electrical_frame =
            float{
                (enc.raw_value - main_configuration.persistent_data.phase_offset )
            } * enc.degrees_per_tick;
    }
    return;
}

#endif // ENCODER_HPP
