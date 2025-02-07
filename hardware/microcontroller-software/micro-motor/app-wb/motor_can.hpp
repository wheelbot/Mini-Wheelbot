/* data.hpp
*
* Copyright (C) 2024 Henrik Hose
* Copyright (C) 2021 Christopher Durand
* Copyright (C) 2018 Raphael Lehmann <raphael@rleh.de>
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

namespace motorCan
{

template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;
class Configuration
{
public:
	// A board is a microcontroller two motors (or less)
	static constexpr uint8_t
	BoardCount =  5;

	// All motors are enumerated from 0 to MotorCount - 1.
	static constexpr uint8_t
	MotorCount = BoardCount * 2;

public:
	// ID of the sync packet
	static constexpr uint16_t
	sync_id = 0x0000;

	// Length of the sync packet
	static constexpr uint8_t
	sync_length = 0;

	// ID of the first motor
	static constexpr uint16_t
	base_id = 0x10;

	// ID of the first motor reply packets
	static constexpr uint16_t
	base_id_reply = 0x80;

	// ID of first motor
	static constexpr uint16_t
	configuration_id = 0x20;

	// ID of first motor
	static constexpr uint16_t
	positioncmd_id = 0x30;

	// ID configuration reply
	static constexpr uint16_t
	configuration_id_reply = 0x90;

};

template<typename Fdcan>
void setupCanFilters(std::optional<uint8_t> boardId)
{
	if ( boardId.has_value()){

		Fdcan::setStandardFilter(1, Fdcan::FilterConfig::Fifo0,
			modm::can::StandardIdentifier(Configuration::base_id + boardId.value()),
			modm::can::StandardMask(0x7ff));

		Fdcan::setStandardFilter(2, Fdcan::FilterConfig::Fifo0,
			modm::can::StandardIdentifier(Configuration::configuration_id + boardId.value()),
			modm::can::StandardMask(0x7ff));

		Fdcan::setStandardFilter(3, Fdcan::FilterConfig::Fifo0,
			modm::can::StandardIdentifier(Configuration::positioncmd_id + boardId.value()),
			modm::can::StandardMask(0x7ff));
	}

	Fdcan::setStandardFilter(0, Fdcan::FilterConfig::Fifo0,
		modm::can::StandardIdentifier(Configuration::sync_id),
		modm::can::StandardMask(0x7ff));

	Fdcan::setStandardFilter(4, Fdcan::FilterConfig::Fifo0,
		modm::can::StandardIdentifier(Configuration::configuration_id),
		modm::can::StandardMask(0x7ff));
}

class TorqueToMotor
{
public:
        void updateFromMessageData(const uint8_t* data)
        {
                u = static_cast<float>(int16_t{(data[0] << 8) | data[1]})/1000.f;
        }
public:
        float
        u;
};

class PositionToMotor
{
public:
        void updateFromMessageData(const uint8_t* data)
        {
                u = static_cast<float>(int16_t{(data[0] << 8) | data[1]})/100.f;
        }
public:

        float
        u;
};

class DataFromMotor
{
public:
        uint8_t toMessageData(uint8_t* a) const
        {
                const int16_t a_fp = static_cast<int16_t>(angle * 100.f);
                a[0] = a_fp >> 8;
                a[1] = a_fp & 0xff;
				return 2;
        }
public:
        // must be in range [-180, 180) degrees
        float angle;
};
struct Sync{};

struct DiagnosticResponseFromMotor
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

struct DiagnosticRequestToMotor{

	enum class Commands : uint8_t{
		SetBoardId 					= 0x0,

		CalibrateEncoder 			= 0x11,
		CalibrateCogging 			= 0x12,
		AbortCalibration			= 0x13,
		SetCoggingCalibrationToZero	= 0x14,

		SaveConfigToFlash			= 0x21,
		LoadConfigFromFlash			= 0x22,
		SetAutoReload				= 0x23,
		ResetAutoReload				= 0x24,

		DebugDisableRegularPrinting	= 0x30,
		DebugModeDefault 			= 0x31,
		DebugModeScopeIdq 			= 0x32,
		DebugModeScopePosIq 		= 0x33,

		ReturnCoggingData 			= 0x41,
		ReturnCoggingEncoderOffset	= 0x42,
		ReturnHardwareUuid			= 0x43,
		ReturnCanId					= 0x44,
		ReturnRawEncoder 			= 0x45,
		ReturnSpecialActionStatus	= 0x46,
		ReturnEncoderErrorCounter	= 0x47,


		EnableCoggingCompensation 	= 0x91,
		DisableCoggingCompensation 	= 0x92,

	};

	void fromMessageData(uint8_t* a, const size_t len){
		if ( len >= 1 ) command = Commands{a[0]};
		value = 0;
		second_value = 0;
		if ( len >= (1+4) )
			value = (uint32_t{a[1]} << 24) | (uint32_t{a[2]} << 16) | (uint32_t{a[3]} << 8) | uint32_t{a[4]} ;
		if ( len >= (1+4+2) )
			second_value = (uint16_t{a[5]} << 8) | uint16_t{a[6]};
	}
	Commands command{0};
	uint32_t value{0};
	uint16_t second_value{0};
};

using CanMessage = std::variant<std::monostate, Sync, TorqueToMotor, DiagnosticRequestToMotor, PositionToMotor>;

template<typename Fdcan>
CanMessage getCanMessage(std::optional<uint8_t> boardId)
{
	while(Fdcan::isMessageAvailable())
	{
		// MODM_LOG_INFO << "Got Message" << modm::endl;
		modm::can::Message canMessage;
		if (!Fdcan::getMessage(canMessage)) {
			break;
		}
		if (boardId.has_value() && canMessage.getIdentifier() == uint32_t(Configuration::base_id + boardId.value()) && canMessage.getLength() == 2) {
			TorqueToMotor data;
			data.updateFromMessageData(canMessage.data);
			return data;
		} else if (canMessage.getIdentifier() == Configuration::sync_id) {
			return Sync{};
		}
		else if (
			( boardId.has_value() && canMessage.getIdentifier() == uint32_t(Configuration::configuration_id + boardId.value()) )
			|| ( canMessage.getIdentifier() == uint32_t(Configuration::configuration_id) ) ){
			DiagnosticRequestToMotor config;
			config.fromMessageData(canMessage.data, canMessage.getLength());
			return config;
		}
		else if (boardId.has_value() && canMessage.getIdentifier() == uint32_t(Configuration::positioncmd_id + boardId.value()) && canMessage.getLength() == 2) {
			PositionToMotor data;
			data.updateFromMessageData(canMessage.data);
			return data;
		}
	}

	return std::monostate{};
}

template<typename Fdcan>
void sendResponse(const DataFromMotor& data, std::optional<uint8_t>  boardId)
{
	if (!boardId.has_value()) return;
	modm::can::Message canMessage(Configuration::base_id_reply + boardId.value(), 2);
	canMessage.setExtended(false);
	auto len = data.toMessageData(canMessage.data);
	canMessage.setLength(len);
	Fdcan::sendMessage(canMessage);
}

template<typename Fdcan>
void sendResponse(const DiagnosticResponseFromMotor& data, std::optional<uint8_t> boardId)
{
	modm::can::Message canMessage(Configuration::configuration_id_reply + boardId.value_or(0), 8);
	canMessage.setExtended(false);
	auto len = data.toMessageData(canMessage.data);
	canMessage.setLength(len);
	Fdcan::sendMessage(canMessage);
}

}
