/*
 * Copyright (C) 2019 Raphael Lehmann <raphael@rleh.de>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef MOTOR_CAN_SLAVE_HPP
#define MOTOR_CAN_SLAVE_HPP

#include <stdint.h>

#include "motor_configuration.hpp"

#include <array>
#include <modm/processing/timer/timeout.hpp>
#include <modm/architecture/interface/can.hpp>
#include <modm/architecture/interface/atomic_lock.hpp>
#include <modm/platform/can/can_filter.hpp>

namespace motorCan
{

/**
 * Communicate at high speed with all motors connected by High Speed CAN.
 *
 * update() must be called at a frequency of 2 milliseconds or faster.
 */
template < typename CAN_BUS, typename MotorBoard >
class MotorCanSlave
{
public:

	static void
	initialize(uint8_t boardId);

	static void update();

	// Sample encoder and current values
	static void sampleMotors();

	// Write received values to the bridges on this board
	static void updateMotors();

private:
	// Shadow registers of PWM, encoder, current (target, actual)
	class DataToMotor
	{
	public:
		void updateFromMessageData(uint8_t* data)
		{
			pwmM1 = (data[0] << 8) | data[1];
			pwmM2 = (data[2] << 8) | data[3];
			currentLimitM1 = (data[4] << 8) | data[5];
			currentLimitM2 = (data[6] << 8) | data[7];
		}
	public:
		// Pulse-width ratio of 10 bit PWM register
		// -1023 .. -1: negative PWM
		//           0: no PWM
		// +1 .. +1023: positive PWM
		int16_t
		pwmM1;
		int16_t
		pwmM2;

		// Current limit that is limited by the motor hardware
		uint16_t
		currentLimitM1;
		uint16_t
		currentLimitM2;
	} __attribute__((packed));

	class DataFromMotor
	{
	public:
		void toMessageData(uint8_t* a)
		{
			a[0] = encoderCounterRawM1 >> 8;
			a[1] = encoderCounterRawM1 & 0xff;
			a[2] = encoderCounterRawM2 >> 8;
			a[3] = encoderCounterRawM2 & 0xff;
			a[4] = currentM1 >> 8;
			a[5] = currentM1 & 0xff;
			a[6] = currentM2 >> 8;
			a[7] = currentM2 & 0xff;
		}
	public:
		uint16_t encoderCounterRawM1;
		uint16_t encoderCounterRawM2;

		// Actual combined motor current in milliamperes
		// Negative current means regenerative breaking
		int16_t currentM1;
		int16_t currentM2;
	} __attribute__((packed));

	static inline uint8_t boardId;

	static inline DataToMotor dataToMotor;
	static inline DataFromMotor dataFromMotor;

private:
	static inline void
	transmit();

	static inline void
	receive();

	static inline modm::ShortTimeout aliveTimer;
};

} // namespace motorCan

#include "slave_impl.hpp"

#endif /* MOTOR_CAN_SLAVE_HPP */
