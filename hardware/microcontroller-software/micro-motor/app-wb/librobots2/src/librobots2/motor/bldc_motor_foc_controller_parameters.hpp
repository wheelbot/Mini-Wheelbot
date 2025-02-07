#ifndef CALCULATE_CONTROLLER_PARAMETERS_HPP
#define CALCULATE_CONTROLLER_PARAMETERS_HPP

#include <modm/math/filter/pid.hpp>

namespace librobots2::motor
{
/// @brief Tuning PI controller for bldc foc
// https://e2e.ti.com/blogs_/b/industrial_strength/posts/teaching-your-pi-controller-to-behave-part-ii

struct BldcMotorParameters{
    float inductance;                           // [H] equivalent DC inductance
    float resistance;                           // [Ohm] equivalent DC resistance
    float electrical_per_mechanical_rev;    // []
};

struct CurrentSenseParameters{
    float reference_voltage;            // [V]
    float amp_gain;                     // [V/V]
    float adc_resolution;               // []
    float shunt_resistance;             // [Ohm]
};

struct ControlLoopParameters{
    float current_control_loop_rate;    // [Hz]
    float output_range;
    float integral_output_range;
};


class PhysicalMotorModel
{
public:

    using PidParameters = modm::Pid<float>::Parameter;

    PhysicalMotorModel(
        BldcMotorParameters bldcMotorParameters,
        CurrentSenseParameters currentSenseParameters,
        ControlLoopParameters controlLoopParameters):
        bldcMotorParameters_(bldcMotorParameters),
        currentSenseParameters_(currentSenseParameters),
        controlLoopParameters_(controlLoopParameters)
        {
            calculateAmpPerAdcReading();
        }

    PidParameters calculatePidParameters(const float bandwidth, const float supply_voltage){
        const float Kb = bldcMotorParameters_.resistance / bldcMotorParameters_.inductance;
        const float Ka = 2.f * std::numbers::pi_v<float> * bandwidth;
        const float output_per_volt = 1 / supply_voltage;
        const float Ki = output_per_volt * Ka * Kb / controlLoopParameters_.current_control_loop_rate;
	    const float Kp = output_per_volt * Ka;
        pidParameters_ = PidParameters{ Kp, Ki, 0, controlLoopParameters_.integral_output_range/Ki, controlLoopParameters_.output_range};
        return pidParameters_;
    };

    constexpr
    float calculateAmpPerAdcReading(){
        amp_per_adc_reading_ = currentSenseParameters_.reference_voltage/(currentSenseParameters_.shunt_resistance * currentSenseParameters_.amp_gain * currentSenseParameters_.adc_resolution);
        return amp_per_adc_reading_;
    }

    constexpr
    inline
    float getAmpPerAdcReading()
    {
        return amp_per_adc_reading_;
    }


private:
    BldcMotorParameters bldcMotorParameters_;
    CurrentSenseParameters currentSenseParameters_;
    ControlLoopParameters controlLoopParameters_;
    PidParameters pidParameters_;
    float amp_per_adc_reading_;
};

// constexpr float
// amp_per_adc_reading(){
//     return V_ref / ( R_shunt * amp_gain * adc_resolution );
// }

// modm::Pid<float>::Parameter
// calculatePidParameters(BldcMotorParameters, CurrentSenseParameters, ControlLoopParameters){
//     const auto Kb = R/L;
//     const auto Ka = 2.f * std::numbers::pi*L * f_BW;
//     const auto output_per_volt = output_range / V_supply;
//     const float Ki = output_per_volt * Ka * Kb / f_loop * amp_per_adc_reading<>();
//     const float Kp = output_per_volt * Ka * amp_per_adc_reading<>();
//     return modm::Pid<float>::Parameter{ Kp, Ki, 0, output_range/Ki/4, 1*output_range};
// }

} // namesapce librobots2::motor

#endif
