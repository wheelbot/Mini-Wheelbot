/*
 * Copyright (C) 2019 Raphael Lehmann <raphael@rleh.de>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef MOTOR_CAN_SLAVE_HPP
#error "Do not include this file directly. Include slave.hpp instead."
#endif

#include <type_traits>
#include <limits>
#include <modm/platform/can/can_2.hpp>

namespace motorCan {

template < typename CAN_BUS, typename MotorBoard >
void
MotorCanSlave< CAN_BUS, MotorBoard >::initialize(uint8_t _boardId)
{
	boardId = _boardId;

	using CanFilter = modm::platform::CanFilter;

	MODM_LOG_DEBUG << "[motorCan] boardId = " << boardId << modm::endl;

	CanFilter::setStartFilterBankForCan2(14);

	if constexpr(std::is_same_v<CAN_BUS, modm::platform::Can2>) {
		// Only receive sync message
		CanFilter::setFilter(0, CanFilter::FIFO0,
			CanFilter::StandardIdentifier(0),
			CanFilter::StandardFilterMask(0x7ff));

		CanFilter::setFilter(14, CanFilter::FIFO0,
				CanFilter::StandardIdentifier(Configuration::sync_id),
				CanFilter::StandardFilterMask(0x7ff));
		CanFilter::setFilter(15, CanFilter::FIFO0,
				CanFilter::StandardIdentifier(Configuration::base_id + boardId),
				CanFilter::StandardFilterMask(0x7ff));
	} else {
		// Only receive sync message
		CanFilter::setFilter(14, CanFilter::FIFO0,
			CanFilter::StandardIdentifier(0),
			CanFilter::StandardFilterMask(0x7ff));

		CanFilter::setFilter(0, CanFilter::FIFO0,
				CanFilter::StandardIdentifier(Configuration::sync_id),
				CanFilter::StandardFilterMask(0x7ff));
		CanFilter::setFilter(1, CanFilter::FIFO0,
				CanFilter::StandardIdentifier(Configuration::base_id + boardId),
				CanFilter::StandardFilterMask(0x7ff));
	}
	MODM_LOG_DEBUG << "[motorCan] Listening to CAN messages with IDsß 0x" << modm::hex << static_cast<uint8_t>(Configuration::base_id + boardId);
	MODM_LOG_DEBUG << modm::ascii << " and 0x" << modm::hex << static_cast<uint8_t>(Configuration::sync_id) << modm::ascii << modm::endl;

	aliveTimer.restart(0);
}

// ----------------------------------------------------------------------------

template < typename CAN_BUS, typename MotorBoard >
void
MotorCanSlave< CAN_BUS, MotorBoard >::update()
{
	receive();
}

// ----------------------------------------------------------------------------

template < typename CAN_BUS, typename MotorBoard >
void
MotorCanSlave< CAN_BUS, MotorBoard >::sampleMotors() {
	dataFromMotor.encoderCounterRawM1 = MotorBoard::getRawEncoderValue(MotorBoard::Motor::M1);
	dataFromMotor.encoderCounterRawM2 = MotorBoard::getRawEncoderValue(MotorBoard::Motor::M2);
	dataFromMotor.currentM1 = MotorBoard::getCurrent(MotorBoard::Motor::M1);
	dataFromMotor.currentM2 = MotorBoard::getCurrent(MotorBoard::Motor::M2);
};

// ----------------------------------------------------------------------------

template < typename CAN_BUS, typename MotorBoard >
void
MotorCanSlave< CAN_BUS, MotorBoard >::updateMotors() {
	if(dataToMotor.pwmM1 == std::numeric_limits<int16_t>::min()) {
		MotorBoard::disable(MotorBoard::Motor::M1);
	}
	else {
		MotorBoard::setPwm(MotorBoard::Motor::M1, dataToMotor.pwmM1);
	}
	if(dataToMotor.pwmM2 == std::numeric_limits<int16_t>::min()) {
		MotorBoard::disable(MotorBoard::Motor::M2);
	}
	else {
		MotorBoard::setPwm(MotorBoard::Motor::M2, dataToMotor.pwmM2);
	}
	MotorBoard::setCurrentLimit(MotorBoard::Motor::M1, dataToMotor.currentLimitM1);
	MotorBoard::setCurrentLimit(MotorBoard::Motor::M2, dataToMotor.currentLimitM2);
};

// ----------------------------------------------------------------------------

template < typename CAN_BUS, typename MotorBoard >
void
MotorCanSlave< CAN_BUS, MotorBoard >::receive()
{
	while (CAN_BUS::isMessageAvailable())
	{
		modm::can::Message message;
		CAN_BUS::getMessage(message);

		// Check if sync packet received
		if ((message.identifier == Configuration::sync_id) && (message.length == Configuration::sync_length)) {
			MODM_LOG_DEBUG << "Sync received." << modm::endl;

			// Sample synchronously
			sampleMotors(); // Hardware dependent

			// Then reply instantly
			// No collisions occur due to the arbitration mechanism of CAN_BUS bus
			modm::can::Message msg(Configuration::base_id_reply + boardId, sizeof(DataFromMotor));
			msg.setExtended(false);
			dataFromMotor.toMessageData(msg.data);
			bool ret = CAN_BUS::sendMessage(msg);
			if (ret == false) {
				MODM_LOG_ERROR << "Unable to reply to sync." << modm::endl;
			}

		}
		else
		{
			// Check if new motor data was received
			// CAN filtering guarantees that only messages with motors on the board are received.
			if ((message.identifier ==  static_cast<uint32_t>(Configuration::base_id + boardId)) && (message.length == sizeof(DataToMotor)))
			{
				MODM_LOG_DEBUG << "Motor data received. Message id " << message.identifier << modm::endl;

				dataToMotor.updateFromMessageData(message.data);

				updateMotors();

				// Valid frame received, motorCan master seems to be up
				aliveTimer.restart(100);
			}
		}
	}
}

} // namespace motorCan
