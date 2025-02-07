/* bldc_motor_foc_impl.hpp
*
* Copyright (C) 2021 Christopher Durand
*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef LIBMOTOR_BLDC_MOTOR_FOC_HPP
#error "Do not use this file directly, include 'bldc_motor_foc.hpp' instead!"
#endif

#include "svm.hpp"

namespace librobots2::motor
{

template<typename MotorBridge>
BldcMotorFoc<MotorBridge>::BldcMotorFoc()
{
    //disableMotor();
}

template<typename MotorBridge>
BldcMotorFoc<MotorBridge>::~BldcMotorFoc()
{
    disableMotor();
}

template<typename MotorBridge>
void BldcMotorFoc<MotorBridge>::setSetpoint(float current)
{
    setpointQ_ = current;
}


template<typename MotorBridge>
void BldcMotorFoc<MotorBridge>::setSetpoint(float current, float feedforward)
{
    setpointQ_ = current;
    feedforwardQ_ = feedforward;
}


template<typename MotorBridge>
void BldcMotorFoc<MotorBridge>::setFluxCurrentSetpoint(float current)
{
    setpointD_ = current;
}

template<typename MotorBridge>
void BldcMotorFoc<MotorBridge>::setControllerParameters(const PidParameters& parameters)
{
    controllerD_.setParameter(parameters);
    controllerQ_.setParameter(parameters);
}

template<typename MotorBridge>
void BldcMotorFoc<MotorBridge>::update()
{
    float sine{}, cosine{};
    arm_sin_cos_f32(motorAngle_, &sine, &cosine);
    const auto currentD =  cosine * currentAlpha_ + sine   * currentBeta_;
    const auto currentQ =  -sine  * currentAlpha_ + cosine * currentBeta_;

    controllerD_.update(currentD - setpointD_);
    controllerQ_.update(currentQ - setpointQ_);

    const float voltageD = controllerD_.getValue();
    const float voltageQ = controllerQ_.getValue() + feedforwardQ_;

    // transform voltage back to alpha/beta coordinates
    const auto voltageAlpha = cosine * voltageD -   sine * voltageQ;
    const auto voltageBeta  =   sine * voltageD + cosine * voltageQ;

    setSvmOutput<MotorBridge>(-voltageAlpha, -voltageBeta);
}

template<typename MotorBridge>
void BldcMotorFoc<MotorBridge>::disable()
{
    disableMotor();
}

template<typename MotorBridge>
void BldcMotorFoc<MotorBridge>::enable()
{
    enableMotor();
}

template<typename MotorBridge>
void BldcMotorFoc<MotorBridge>::disableMotor()
{
    MotorBridge::configure(PhaseConfig::HiZ);
}

template<typename MotorBridge>
void BldcMotorFoc<MotorBridge>::enableMotor()
{
    MotorBridge::configure(PhaseConfig::Pwm);
    MotorBridge::setCompareValue(0);
}

template<typename MotorBridge>
void BldcMotorFoc<MotorBridge>::setCurrentMeasurement(float alpha, float beta)
{
    currentAlpha_ = alpha;
    currentBeta_ = beta;
}

template<typename MotorBridge>
void BldcMotorFoc<MotorBridge>::setMotorAngle(float angleDegrees)
{
    motorAngle_ = angleDegrees;
}

}
