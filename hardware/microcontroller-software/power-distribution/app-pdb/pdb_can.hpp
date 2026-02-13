/* pdb_can.hpp
*
* Copyright (C) 2025 Henrik Hose
* Copyright (C) 2025 Jan Weisgerber
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

#pragma once

#include <stdint.h>
#include <modm/io/iostream.hpp>

#include <array>
#include <variant>

namespace pdbCan {

template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;
class Configuration
{

public:
	// ID of the sync packet
	static constexpr uint16_t
	sync_id = 0x41;

	// Length of the sync packet
	static constexpr uint8_t
	sync_length = 0;

	// ID of the pdb
	static constexpr uint16_t
	base_id = 0x40;

	// ID of the pdb data reply packets
	static constexpr uint16_t
	data_id_reply = 0x42;

	// ID of the first pdb reply packets
	static constexpr uint16_t
	base_id_reply = 0x49;

};

template<typename Fdcan>
void setupCanFilters()
{
	Fdcan::setStandardFilter(1, Fdcan::FilterConfig::Fifo0,
		modm::can::StandardIdentifier(Configuration::base_id),
		modm::can::StandardMask(0x7ff));

	Fdcan::setStandardFilter(0, Fdcan::FilterConfig::Fifo0,
		modm::can::StandardIdentifier(Configuration::sync_id),
		modm::can::StandardMask(0x7ff));
}

struct RequestToPdb{
    enum class Commands : uint8_t{
        disableMotors           = 0x01,
        enableMotors            = 0x02,
        processBitmask          = 0x03,
    };

    void fromMessageData(uint8_t* a, const size_t len){
		if ( len >= 1 ) command = Commands{a[0]};
		value = 0;
		second_value = 0;
		if ( len >= (1+3) )
			value = (uint32_t{a[1]} << 16) |(uint32_t{a[2]} << 8) | uint32_t{a[3]} ;
		if ( len >= (1+4) )
			value = (uint32_t{a[1]} << 24) | (uint32_t{a[2]} << 16) | (uint32_t{a[3]} << 8) | uint32_t{a[4]} ;
		if ( len >= (1+4+2) )
			second_value = (uint16_t{a[5]} << 8) | uint16_t{a[6]};
	}
	Commands command{0};
	uint32_t value{0};
	uint16_t second_value{0};

};

struct Sync{};

using CanMessage = std::variant<std::monostate, RequestToPdb, Sync>;


template<typename Fdcan>
CanMessage getCanMessage()
{
	while(Fdcan::isMessageAvailable())
	{
		// MODM_LOG_INFO << "Got Message" << modm::endl;
		modm::can::Message canMessage;
		if (!Fdcan::getMessage(canMessage)) {
			break;
		}
		if (( canMessage.getIdentifier() == uint32_t(Configuration::base_id) )){
			RequestToPdb config;
			config.fromMessageData(canMessage.data, canMessage.getLength());
			return config;
		} else if (canMessage.getIdentifier() == Configuration::sync_id) {
			return Sync{};
		}
	}

	return std::monostate{};
}

class DataFromPDB
{
public:
    uint8_t toMessageData(uint8_t* a) const
    {
        // Encode voltage as fixed-point (x100)
        const int16_t v_fp = static_cast<int16_t>(voltage * 100.f);
        a[0] = v_fp >> 8;
        a[1] = v_fp & 0xff;

        // Encode current as fixed-point (x100)
        const int16_t i_fp = static_cast<int16_t>(current * 100.f);
        a[2] = i_fp >> 8;
        a[3] = i_fp & 0xff;

        return 4;
    }

public:
    float voltage;   // in Volts
    float current;   // in Amps
};

struct ResponseFromPDB
{
	enum class ControlWord : uint8_t{
		Data = 1,
	};

	uint8_t toMessageData(uint8_t* a) const
	{
		a[0] = static_cast<uint8_t>(control_word);
		return 1 + std::visit( overloaded {
			[](std::monostate){return 0;},
			[&](const float d){
				uint32_t data; memcpy(&data, &d, 4);
				a[1] = (data >> 24) & 0xff;
				a[2] = (data >> 16) & 0xff;
				a[3] = (data >>  8) & 0xff;
				a[4] = data & 0xff;
				return 4;},
			[&](const uint8_t d){
				a[1] = d & 0xff;
				return 1;
				},
			[&](const uint16_t d){
				a[1] = (d >> 8) & 0xff;
				a[2] = d & 0xff;
				return 2;
				},
			[&](const uint32_t d){
				a[1] = (d >> 24) & 0xff;
				a[2] = (d >> 16) & 0xff;
				a[3] = (d >>  8) & 0xff;
				a[4] = d & 0xff;
				return 4;
				},
			[&](const std::pair<uint16_t, float> d){
				a[1] = (d.first >> 8) & 0xff;
				a[2] = d.first & 0xff;
				uint32_t data; memcpy(&data, &d.second, 4);
				a[3] = (data >> 24) & 0xff;
				a[4] = (data >> 16) & 0xff;
				a[5] = (data >>  8) & 0xff;
				a[6] = data & 0xff;
				return 6;
				},
		},
		data
		);
	}

	ControlWord control_word;
	std::variant<std::monostate, float, uint8_t, uint16_t, uint32_t, std::pair<uint16_t, float>> data;
};


template<typename Fdcan>
void sendResponse(ResponseFromPDB data)
{
	modm::can::Message canMessage(Configuration::base_id_reply, 8);
	canMessage.setExtended(false);
	auto len = data.toMessageData(canMessage.data);
	canMessage.setLength(len);
	Fdcan::sendMessage(canMessage);
}

template<typename Fdcan>
void sendResponse(const DataFromPDB& data)
{
	modm::can::Message canMessage(Configuration::data_id_reply, 4);
	canMessage.setExtended(false);
	auto len = data.toMessageData(canMessage.data);
	canMessage.setLength(len);
	Fdcan::sendMessage(canMessage);
}


}
