// INSERT MIT LICENSE HEADER
// adapted from https://github.com/odriverobotics/ODrive

#ifndef LIBROBOTS2_MOTOR_SVM_HPP
#define LIBROBOTS2_MOTOR_SVM_HPP

#include <utility>
#include <tuple>
#include "motor_bridge.hpp"
#include <numbers>
#include <cmsis/dsp/arm_math.h>

namespace librobots2::motor
{

constexpr auto svm(float alpha, float beta)
    -> std::tuple<float, float, float, bool>
{
    constexpr auto one_by_sqrt3 = std::numbers::inv_sqrt3_v<float>;
    constexpr float two_by_sqrt3 = 2.f * std::numbers::inv_sqrt3_v<float>;

    float tA, tB, tC;
    int Sextant;

    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            //quadrant I
            if (one_by_sqrt3 * beta > alpha)
                Sextant = 2; //sextant v2-v3
            else
                Sextant = 1; //sextant v1-v2
        } else {
            //quadrant II
            if (-one_by_sqrt3 * beta > alpha)
                Sextant = 3; //sextant v3-v4
            else
                Sextant = 2; //sextant v2-v3
        }
    } else {
        if (alpha >= 0.0f) {
            //quadrant IV
            if (-one_by_sqrt3 * beta > alpha)
                Sextant = 5; //sextant v5-v6
            else
                Sextant = 6; //sextant v6-v1
        } else {
            //quadrant III
            if (one_by_sqrt3 * beta > alpha)
                Sextant = 4; //sextant v4-v5
            else
                Sextant = 5; //sextant v5-v6
        }
    }

    switch (Sextant) {
        // sextant v1-v2
        case 1: {
            // Vector on-times
            float t1 = alpha - one_by_sqrt3 * beta;
            float t2 = two_by_sqrt3 * beta;

            // PWM timings
            tA = (1.0f - t1 - t2) * 0.5f;
            tB = tA + t1;
            tC = tB + t2;
        } break;

        // sextant v2-v3
        case 2: {
            // Vector on-times
            float t2 = alpha + one_by_sqrt3 * beta;
            float t3 = -alpha + one_by_sqrt3 * beta;

            // PWM timings
            tB = (1.0f - t2 - t3) * 0.5f;
            tA = tB + t3;
            tC = tA + t2;
        } break;

        // sextant v3-v4
        case 3: {
            // Vector on-times
            float t3 = two_by_sqrt3 * beta;
            float t4 = -alpha - one_by_sqrt3 * beta;

            // PWM timings
            tB = (1.0f - t3 - t4) * 0.5f;
            tC = tB + t3;
            tA = tC + t4;
        } break;

        // sextant v4-v5
        case 4: {
            // Vector on-times
            float t4 = -alpha + one_by_sqrt3 * beta;
            float t5 = -two_by_sqrt3 * beta;

            // PWM timings
            tC = (1.0f - t4 - t5) * 0.5f;
            tB = tC + t5;
            tA = tB + t4;
        } break;

        // sextant v5-v6
        case 5: {
            // Vector on-times
            float t5 = -alpha - one_by_sqrt3 * beta;
            float t6 = alpha - one_by_sqrt3 * beta;

            // PWM timings
            tC = (1.0f - t5 - t6) * 0.5f;
            tA = tC + t5;
            tB = tA + t6;
        } break;

        // sextant v6-v1
        case 6: {
            // Vector on-times
            float t6 = -two_by_sqrt3 * beta;
            float t1 = alpha + one_by_sqrt3 * beta;

            // PWM timings
            tA = (1.0f - t6 - t1) * 0.5f;
            tC = tA + t1;
            tB = tC + t6;
        } break;
    }

    bool result_valid =
            tA >= 0.0f && tA <= 1.0f
        && tB >= 0.0f && tB <= 1.0f
        && tC >= 0.0f && tC <= 1.0f;
    return {tA, tB, tC, result_valid};
}

template<typename T = float>
constexpr std::tuple<T,T> clipAB(T alpha, T beta){
    constexpr float two_by_sqrt3 = 2.f / sqrt(3.f);
    // maximum uniformly achievable output vector is sqrt(3)/2 long
    // https://en.wikipedia.org/wiki/Space_vector_modulation
    const auto radius_squared = alpha*alpha+beta*beta;
    auto alpha_clip = alpha;
    auto beta_clip = beta;
    if ( radius_squared >= 0.75f){ // this is (sqrt(3)/2)^2
        const auto radius = std::sqrt(radius_squared)*two_by_sqrt3;
        alpha_clip/=radius;
        beta_clip/=radius;
    }
    return {alpha_clip, beta_clip};
}

template<typename MotorBridge>
void setSvmOutput(float alpha, float beta)
{
    alpha = std::max(std::min(alpha, 1.0f), -1.0f);
    beta = std::max(std::min(beta, 1.0f), -1.0f);

    const auto [scaled_alpha, scaled_beta] = clipAB<float>(alpha, beta);
    const auto [a, b, c, valid] = svm(scaled_alpha, scaled_beta);
    const uint16_t aDutyCycle = std::min<float>(a * MotorBridge::MaxPwm, MotorBridge::MaxPwm);
    const uint16_t bDutyCycle = std::min<float>(b * MotorBridge::MaxPwm, MotorBridge::MaxPwm);
    const uint16_t cDutyCycle = std::min<float>(c * MotorBridge::MaxPwm, MotorBridge::MaxPwm);

    MotorBridge::setCompareValue(Phase::PhaseU, aDutyCycle);
    MotorBridge::setCompareValue(Phase::PhaseV, bDutyCycle);
    MotorBridge::setCompareValue(Phase::PhaseW, cDutyCycle);
}

template<typename MotorBridge>
void setSvmOutputMagnitudeAngle(float magnitude, float angle)
{
	float sine{};
	float cosine{};
	arm_sin_cos_f32(angle, &sine, &cosine);

	sine *= magnitude;
	cosine *= magnitude;

	setSvmOutput<MotorBridge>(cosine, sine);
}

constexpr auto clarkeTransform(float u, float v)
	-> std::tuple<float, float>
{
	constexpr float two_by_sqrt3 = 2.f * std::numbers::inv_sqrt3_v<float>;

	return {u, u * std::numbers::inv_sqrt3_v<float> + v * two_by_sqrt3};
}


constexpr auto clarkeTransform(float u, float v, float w)
	-> std::tuple<float, float>
{
	// constexpr float two_by_sqrt3 = 2.f * std::numbers::inv_sqrt3_v<float>;
	constexpr float one_by_sqrt3 = std::numbers::inv_sqrt3_v<float>;

	return {0.66666666f*u-0.3333333f*v-0.3333333f*w,
        one_by_sqrt3*v-one_by_sqrt3*w};
}

auto parkTransform(float a, float b, float electrical_angle_degrees)
    -> std::tuple<float, float, float, float>
{
    float sine{}, cosine{};
    arm_sin_cos_f32(electrical_angle_degrees, &sine, &cosine);
    return { cosine * a +   sine * b,
            -sine * b  + cosine * b,
             sine, cosine};
}

}

#endif // LIBROBOTS2_MOTOR_SVM_HPP
