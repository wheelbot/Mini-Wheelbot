#ifndef WHEELBOT_HARDWARE_HPP
#define WHEELBOT_HARDWARE_HPP

#include <micro-motor/hardware.hpp>
#include <librobots2/motor/bldc_motor_foc.hpp>
#include <librobots2/motion/encoder/atomic_encoder.hpp>
#include <librobots2/motor/bldc_motor_foc_controller_parameters.hpp>

namespace wheelbot{

using BldcMotor = librobots2::motor::BldcMotorFoc<Board::Motor>;

using librobots2::motor::BldcMotorParameters;
using librobots2::motor::CurrentSenseParameters;
using librobots2::motor::ControlLoopParameters;
using librobots2::motor::PhysicalMotorModel;

constexpr
BldcMotorParameters mn4006Parameters{
	.inductance = 80.e-6f,
	.resistance = 150.e-3f,
	.electrical_per_mechanical_rev = 12.f};

constexpr
CurrentSenseParameters wbPcb{
	.reference_voltage = 2.9f,
	.amp_gain=20,
	.adc_resolution=float(1<<12),
	.shunt_resistance=3.e-3f};

constexpr
ControlLoopParameters controlLoopParameters{
	.current_control_loop_rate = 83.e3f,
	.output_range= 1.f,
	.integral_output_range=0.4f};

int16_t current_U_offset;
int16_t current_V_offset;

PhysicalMotorModel physicalMotor(mn4006Parameters, wbPcb, controlLoopParameters);
BldcMotor motor;

constexpr uint16_t encoder_resolution = 2000;
constexpr bool encoder_inverted = false;
AtomicEncoder<float, Board::Encoder::ABI::getEncoderRaw> encoder(
	encoder_resolution,
	encoder_inverted
	);



};


#endif
