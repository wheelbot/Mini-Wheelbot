// MIT License

// Copyright (c) 2024 Henrik Hose
//               2024 Jan Weisgerber

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP


#include <eigen3/Eigen/Dense>
#include "wheelbot/sensitivity_20240528-181320/93.eqx/neural_network.hpp"
#include "wheelbot/action_20240528-132835/190.eqx/neural_network.hpp"
#include "wheelbot/config.hpp"

template <typename Derived>
auto clip(const Eigen::MatrixBase<Derived>& vector, typename Derived::Scalar lower, typename Derived::Scalar upper) {
    return vector.cwiseMax(lower).cwiseMin(upper);
}

template <
    size_t LEN = 5
>
Eigen::RowVector<float, LEN> readCSV(const std::string& filename) {
    Eigen::RowVector<float, LEN> result;

    // Open the CSV file
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the CSV file" << std::endl;
        return result;
    }

    // Read the CSV values
    std::string line;
    std::getline(file, line);
    std::istringstream ss(line);
    std::string token;

    int i = 0;
    while (std::getline(ss, token, ',') && i < LEN) {
        result(i) = std::stof(token);
        i++;
    }

    // Check if we read exactly 4 parameters
    if (i != LEN) {
        std::cerr << "Error: CSV file should contain " << LEN << " parameters" << std::endl;
        result.setZero();
    }

    return result;
}

enum ControllerIdxs : unsigned int
    {ROLL_STANDUP_POS,
    ROLL_STANDUP_NEG,
    ROLL_LAYDOWN_POS,
    ROLL_LAYDOWN_NEG,
    STATE_FEEDBACK,
    PITCH_STANDUP_POS,
    PITCH_STANDUP_NEG,
    PITCH_LAYDOWN_POS,
    PITCH_LAYDOWN_NEG,
    FLIP_STANDUP_ROLL_POS,
    FLIP_STANDUP_ROLL_NEG};


class Abstract_Controller{
public:
    virtual Eigen::Vector2f update(Eigen::Vector<float, 10> x, Eigen::Vector<float, 10> setpoint) {};
    virtual bool AmIDone() {};
    virtual void init() {};
};


class DecoupledRollPitchLqrController: public Abstract_Controller{
public:
    DecoupledRollPitchLqrController(
        Config::BalancingConfig config,
        Config::StandUpConfig standup_config,
        const float u_max              = 0.5,
        const float max_angles         = 0.3 )
        : K_roll(config.K_roll.data()),
            K_pitch(config.K_pitch.data()),
            K_roll_regular(config.K_roll.data()),
            K_pitch_regular(config.K_pitch.data()),
            K_roll_pitch_standup(standup_config.K_roll_pitch_standup.data()),
            K_pitch_pitch_standup(standup_config.K_pitch_pitch_standup.data()),
            K_roll_flip_standup(standup_config.K_roll_flip_standup.data()),
            K_pitch_flip_standup(standup_config.K_pitch_flip_standup.data()),
            pitch_duration_switch_to_regular_operation(standup_config.pitch_duration_switch_to_regular_operation),
            flip_duration_switch_to_regular_operation(standup_config.flip_duration_switch_to_regular_operation),
            K_roll_regular_fallback(config.K_roll.data()),
            K_pitch_regular_fallback(config.K_pitch.data()),
            K_roll_pitch_standup_fallback(standup_config.K_roll_pitch_standup_fallback.data()),
            K_pitch_pitch_standup_fallback(standup_config.K_pitch_pitch_standup_fallback.data()),
            shift_setpoint_drive_wheel_ang(standup_config.shift_setpoint_drive_wheel_ang),
            toggle_state_feedback_shift_setpoint_drive_wheel_pos(standup_config.toggle_state_feedback_shift_setpoint_drive_wheel_pos),
            pitch_standup_shift_setpoint(standup_config.pitch_standup_shift_setpoint),
            u_max(u_max),
            max_angles(max_angles)
    {
    }

    Eigen::Vector2f update(
            Eigen::Vector<float, 10> x,
            Eigen::Vector<float, 10> setpoint = Eigen::Vector<float,10>::Zero()) override {

        Eigen::Vector2f u = Eigen::Vector2f::Zero();
        if ( ( x({ROLL, PITCH}, 1).cwiseAbs().array() > max_angles ).any()) return u;

        Eigen::Vector<float, 10> err_x = x - setpoint;

        wheel_roll_ang +=  err_x(BALANCING_WHEEL_ANGLE) - wheel_roll_ang_last;
        wheel_pitch_ang += err_x(DRIVING_WHEEL_ANGLE) - wheel_pitch_ang_last;

        wheel_roll_ang_last =  err_x(BALANCING_WHEEL_ANGLE);
        wheel_pitch_ang_last = err_x(DRIVING_WHEEL_ANGLE);

        wheel_roll_ang = wheel_roll_ang >  wheel_roll_ang_windup ? wheel_roll_ang - wheel_roll_ang_windup : wheel_roll_ang;
        wheel_roll_ang = wheel_roll_ang < -wheel_roll_ang_windup ? wheel_roll_ang + wheel_roll_ang_windup : wheel_roll_ang;

        wheel_pitch_ang = wheel_pitch_ang >  wheel_pitch_ang_windup ? wheel_pitch_ang - wheel_pitch_ang_windup : wheel_pitch_ang;
        wheel_pitch_ang = wheel_pitch_ang < -wheel_pitch_ang_windup ? wheel_pitch_ang + wheel_pitch_ang_windup : wheel_pitch_ang;

        err_x(BALANCING_WHEEL_ANGLE) = wheel_roll_ang;
        err_x(DRIVING_WHEEL_ANGLE)   = wheel_pitch_ang;

        u(roll_inputs) =  - K_roll  * err_x(roll_states);
        u(pitch_inputs) = - K_pitch * err_x(pitch_states);

        int64_t duration_in_millisec = std::chrono::duration_cast
                <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count()
                 - last_maneuver_starting_time;

        int64_t update_rate = last_duration_in_millisec - duration_in_millisec;

        last_duration_in_millisec = duration_in_millisec;

        if (reset_setpoint) {
            current_setpoint(DRIVING_WHEEL_ANGLE) = x(DRIVING_WHEEL_ANGLE);
            reset_setpoint = false;
        }

        // Setpoint Shift during balancing

        if (toggle_state_feedback_shift_setpoint_drive_wheel_pos &&
            duration_in_millisec > 2000 &&
            duration_in_millisec < 3000)
        {
            // Add the increment to the curren setpoint
            current_setpoint(DRIVING_WHEEL_ANGLE) += shift_setpoint_drive_wheel_ang / (3000 - 2000) * update_rate;
        }

        if (toggle_state_feedback_shift_setpoint_drive_wheel_pos &&
            duration_in_millisec > 5000 &&
            duration_in_millisec < 6000)
        {
            // Add the increment to the curren setpoint
            current_setpoint(DRIVING_WHEEL_ANGLE) -= shift_setpoint_drive_wheel_ang / (6000 - 5000) * update_rate;
        }

        if (toggle_state_feedback_shift_setpoint_drive_wheel_pos &&
            duration_in_millisec > 8000 &&
            duration_in_millisec < 9000)
        {
            // Add the increment to the curren setpoint
            current_setpoint(DRIVING_WHEEL_ANGLE) += shift_setpoint_drive_wheel_ang / (9000 - 8000) * update_rate;
        }

        if (reset_setpoint_pitch_standup)
        {
            std::cout << "Current x -> Driving Wheel Angle:"
                    << x(DRIVING_WHEEL_ANGLE)
                    << '\n';
            current_setpoint(DRIVING_WHEEL_ANGLE) = x(DRIVING_WHEEL_ANGLE) + pitch_standup_shift_setpoint_drive_wheel_ang;
            reset_setpoint_pitch_standup = false;
            std::cout << "Current setpoint -> Driving Wheel Angle:"
                    << current_setpoint(DRIVING_WHEEL_ANGLE)
                    << '\n';
        }

        // Checks whether the state-feedback-/ampc-controller has a duration, default value is set to -1 (no duration limit of control job).
        if (control_job_duration != -1) controller_counter++;

        // After an initially given duration for a pitch standup,
        // the controller gains are updated for regular operation.
        if (toggle_gains_after_pitch_standup && controller_counter == pitch_duration_switch_to_regular_operation) {
            set_gains_regular_operation();
            toggle_gains_after_pitch_standup = false;
        }

        // After an initially given duration for a flip standup,
        // the controller gains are updated for regular operation.
        if (toggle_gains_after_flip_standup && controller_counter == flip_duration_switch_to_regular_operation) {
            set_gains_regular_operation();
            toggle_gains_after_flip_standup = false;
        }

        // When the control job duration limit is reached, the respective variable is set to true.
        if (control_job_duration > 0 && controller_counter >= control_job_duration){
            control_job_done = true;
        }

        return clip(u, -u_max, u_max);
    }

    bool AmIDone() override{
        return control_job_done;
    }

    void init() override{
        last_maneuver_starting_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        controller_counter = 0;
        control_job_done = false;
        wheel_roll_ang=0;
        wheel_roll_ang_last=0;
        wheel_pitch_ang=0;
        wheel_pitch_ang_last=0;
    };

    void set_gains_pitch_standup() {
        std::cout << "gains set for pitch standup" << std::endl;
        K_roll = K_roll_pitch_standup;
        K_pitch = K_pitch_pitch_standup;
        toggle_gains_after_pitch_standup = true;
    }

    void set_gains_flip_standup() {
        std::cout << "gains set for flip standup" << std::endl;
        K_roll = K_roll_flip_standup;
        K_pitch = K_pitch_flip_standup;
        toggle_gains_after_flip_standup = true;
    }

    void set_gains_regular_operation() {
        std::cout << "gains set for regular operation" << std::endl;
        K_roll = K_roll_regular;
        K_pitch = K_pitch_regular;
    }

    void set_fallback_gains_roll_standup() {
        std::cout << "fallback gains set for reset of initial position" << std::endl;
        K_roll = K_roll_regular_fallback;
        K_pitch = K_pitch_regular_fallback;
        toggle_state_feedback_shift_setpoint_drive_wheel_pos = false;
    }

    void set_fallback_gains_pitch_standup() {
        std::cout << "fallback gains set for reset of initial position" << std::endl;
        K_roll = K_roll_pitch_standup_fallback;
        K_pitch = K_pitch_pitch_standup_fallback;
        toggle_gains_after_pitch_standup = true;
        toggle_state_feedback_shift_setpoint_drive_wheel_pos = false;
    }

    void set_setpoint_shift_neg() {
        shift_setpoint_drive_wheel_ang = -abs(shift_setpoint_drive_wheel_ang);
    }

    void set_setpoint_shift_pos() {
        shift_setpoint_drive_wheel_ang = abs(shift_setpoint_drive_wheel_ang);
    }

    void reset_setpoint_pitch_standup_pos() {
        reset_setpoint_pitch_standup = true;
        pitch_standup_shift_setpoint_drive_wheel_ang = -abs(pitch_standup_shift_setpoint);
    }

    void reset_setpoint_pitch_standup_neg() {
        reset_setpoint_pitch_standup = true;
        pitch_standup_shift_setpoint_drive_wheel_ang = abs(pitch_standup_shift_setpoint);
    }

    void set_drive_wheel_setpoint_to_current() {
        reset_setpoint = true;
    }

public:
    Eigen::Matrix<float,1,4> K_roll;
    Eigen::Matrix<float,1,4> K_pitch;
    Eigen::Matrix<float,1,4> K_roll_pitch_standup;
    Eigen::Matrix<float,1,4> K_pitch_pitch_standup;
    Eigen::Matrix<float,1,4> K_roll_regular;
    Eigen::Matrix<float,1,4> K_pitch_regular;
    Eigen::Matrix<float,1,4> K_roll_flip_standup;
    Eigen::Matrix<float,1,4> K_pitch_flip_standup;

    Eigen::Matrix<float,1,4> K_roll_regular_fallback;
    Eigen::Matrix<float,1,4> K_pitch_regular_fallback;
    Eigen::Matrix<float,1,4> K_roll_pitch_standup_fallback;
    Eigen::Matrix<float,1,4> K_pitch_pitch_standup_fallback;

    int64_t last_maneuver_starting_time;
    int64_t last_duration_in_millisec = 0;

    float u_max;

    float max_angles;

    enum StateIdxs : unsigned int {YAW, ROLL, PITCH, YAW_RATE, ROLL_RATE, PITCH_RATE, DRIVING_WHEEL_ANGLE, DRIVING_WHEEL_ANGULAR_VELOCITY, BALANCING_WHEEL_ANGLE, BALANCING_WHEEL_ANGULAR_VELOCITY};
    enum InputIdxs : unsigned int{TAU_DRIVING_WHEEL, TAU_BALANCING_WHEEL};

    static constexpr std::array<int,4> roll_states  {ROLL, ROLL_RATE, BALANCING_WHEEL_ANGLE, BALANCING_WHEEL_ANGULAR_VELOCITY};
    static constexpr std::array<int, 1> roll_inputs {TAU_BALANCING_WHEEL};

    static constexpr std::array<int,4> pitch_states {PITCH, PITCH_RATE, DRIVING_WHEEL_ANGLE, DRIVING_WHEEL_ANGULAR_VELOCITY};
    static constexpr std::array<int, 1> pitch_inputs {TAU_DRIVING_WHEEL};

    float wheel_roll_ang=0;
    float wheel_roll_ang_last=0;
    float wheel_pitch_ang=0;
    float wheel_pitch_ang_last=0;
    static constexpr float wheel_roll_ang_windup=1000;
    static constexpr float wheel_pitch_ang_windup=10000;

    int controller_counter = 0;
    bool control_job_done = false;
    int control_job_duration = -1;
    bool toggle_gains_after_pitch_standup = false;
    bool toggle_gains_after_flip_standup = false;
    int pitch_duration_switch_to_regular_operation = 300;
    int flip_duration_switch_to_regular_operation = 300;

    Eigen::Vector<float, 10> current_setpoint = Eigen::Vector<float,10>::Zero();
    float shift_setpoint_drive_wheel_ang = 0;
    bool toggle_state_feedback_shift_setpoint_drive_wheel_pos = false;

    bool reset_setpoint_pitch_standup = false;
    float pitch_standup_shift_setpoint_drive_wheel_ang = 0;
    float pitch_standup_shift_setpoint = 0;

    bool reset_setpoint = false;
};

class ParameterAdaptiveAMPC : public Abstract_Controller{
public:
    ParameterAdaptiveAMPC(
        const Config::BalancingConfig balancing_config,
        const Config::AmpcConfig ampc_config,
        const Config::StandUpConfig standup_config,
        const float u_max                = 0.35,
        const float max_angles           = 0.8 )
        : K_roll(balancing_config.K_roll.data()),
            K_pitch(balancing_config.K_pitch.data()),
            delta_theta(ampc_config.toArray().data()),
            K_roll_regular(balancing_config.K_roll.data()),
            K_pitch_regular(balancing_config.K_pitch.data()),
            K_roll_pitch_standup(standup_config.K_roll_pitch_standup.data()),
            K_pitch_pitch_standup(standup_config.K_pitch_pitch_standup.data()),
            K_roll_flip_standup(standup_config.K_roll_flip_standup.data()),
            K_pitch_flip_standup(standup_config.K_pitch_flip_standup.data()),
            pitch_duration_switch_to_regular_operation(standup_config.pitch_duration_switch_to_regular_operation),
            flip_duration_switch_to_regular_operation(standup_config.flip_duration_switch_to_regular_operation),
            K_roll_regular_fallback(balancing_config.K_roll.data()),
            K_pitch_regular_fallback(balancing_config.K_pitch.data()),
            K_roll_pitch_standup_fallback(standup_config.K_roll_pitch_standup_fallback.data()),
            K_pitch_pitch_standup_fallback(standup_config.K_pitch_pitch_standup_fallback.data()),
            shift_setpoint_drive_wheel_ang(standup_config.shift_setpoint_drive_wheel_ang),
            toggle_state_feedback_shift_setpoint_drive_wheel_pos(standup_config.toggle_state_feedback_shift_setpoint_drive_wheel_pos),
            pitch_standup_shift_setpoint(standup_config.pitch_standup_shift_setpoint),
            u_max(u_max),
            max_angles(max_angles)
    {

    }

    Eigen::Vector2f update(
            Eigen::Vector<float, 10> x,
            Eigen::Vector<float, 10> setpoint = Eigen::Vector<float,10>::Zero()) override {
        Eigen::Vector2f u = Eigen::Vector2f::Zero();
        if ( ( x({ROLL, PITCH}, 1).cwiseAbs().array() > max_angles ).any()) return u;

        Eigen::Vector<float, 10> err_x = x - setpoint;

        // prestabilizing state-feedback controller
        wheel_roll_ang +=  err_x(BALANCING_WHEEL_ANGLE) - wheel_roll_ang_last;
        wheel_pitch_ang += err_x(DRIVING_WHEEL_ANGLE) - wheel_pitch_ang_last;

        wheel_roll_ang_last =  err_x(BALANCING_WHEEL_ANGLE);
        wheel_pitch_ang_last = err_x(DRIVING_WHEEL_ANGLE);

        wheel_roll_ang = wheel_roll_ang >  wheel_roll_ang_windup ? wheel_roll_ang - wheel_roll_ang_windup : wheel_roll_ang;
        wheel_roll_ang = wheel_roll_ang < -wheel_roll_ang_windup ? wheel_roll_ang + wheel_roll_ang_windup : wheel_roll_ang;

        wheel_pitch_ang = wheel_pitch_ang >  wheel_pitch_ang_windup ? wheel_pitch_ang - wheel_pitch_ang_windup : wheel_pitch_ang;
        wheel_pitch_ang = wheel_pitch_ang < -wheel_pitch_ang_windup ? wheel_pitch_ang + wheel_pitch_ang_windup : wheel_pitch_ang;

        err_x(BALANCING_WHEEL_ANGLE) = wheel_roll_ang;
        err_x(DRIVING_WHEEL_ANGLE)   = wheel_pitch_ang;

        u(roll_inputs) =  - K_roll  * err_x(roll_states);
        u(pitch_inputs) = - K_pitch * err_x(pitch_states);

        // err_x(wheel_angles) = clip(err_x(wheel_angles), -2.f*std::numbers::pi_v<float>, 2.f*std::numbers::pi_v<float>);

        auto err_x_resort_nn = err_x(states_nn);

        // nominal AMPC actions
        const auto action_input_normalized = embedded_nn_inference::action::normalize_input(err_x_resort_nn);
        const auto action_output_normalized = embedded_nn_inference::action::call_nn(action_input_normalized);
        const auto action_output = embedded_nn_inference::action::denormalize_output(action_output_normalized);

        // sensitivity correction
        const auto sensitivity_input_normalized = embedded_nn_inference::sensitivity::normalize_input(err_x_resort_nn);
        const auto sensitivity_output_normalized = embedded_nn_inference::sensitivity::call_nn(sensitivity_input_normalized);
        const auto sensitivity_output = embedded_nn_inference::sensitivity::denormalize_output(sensitivity_output_normalized);
        const auto sensitivity_output_reshaped = embedded_nn_inference::sensitivity::reshape_output(sensitivity_output);
        const auto sensitivity_action_correction = sensitivity_output_reshaped * delta_theta;

        auto u_ampc = action_output + sensitivity_action_correction;
        u += clip(u_ampc, -0.1, 0.1);

        int64_t duration_in_millisec = std::chrono::duration_cast
                <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count()
                 - last_maneuver_starting_time;

        int64_t update_rate = last_duration_in_millisec - duration_in_millisec;

        last_duration_in_millisec = duration_in_millisec;

        if (reset_setpoint) {
            current_setpoint(DRIVING_WHEEL_ANGLE) = x(DRIVING_WHEEL_ANGLE);
            reset_setpoint = false;
        }

        // Setpoint Shift during balancing

        if (toggle_state_feedback_shift_setpoint_drive_wheel_pos &&
            duration_in_millisec > 2000 &&
            duration_in_millisec < 3000)
        {
            // Add the increment to the curren setpoint
            current_setpoint(DRIVING_WHEEL_ANGLE) += shift_setpoint_drive_wheel_ang / (3000 - 2000) * update_rate;
        }

        if (toggle_state_feedback_shift_setpoint_drive_wheel_pos &&
            duration_in_millisec > 5000 &&
            duration_in_millisec < 6000)
        {
            // Add the increment to the curren setpoint
            current_setpoint(DRIVING_WHEEL_ANGLE) -= shift_setpoint_drive_wheel_ang / (6000 - 5000) * update_rate;
        }

        if (toggle_state_feedback_shift_setpoint_drive_wheel_pos &&
            duration_in_millisec > 8000 &&
            duration_in_millisec < 9000)
        {
            // Add the increment to the curren setpoint
            current_setpoint(DRIVING_WHEEL_ANGLE) += shift_setpoint_drive_wheel_ang / (9000 - 8000) * update_rate;
        }

        if (reset_setpoint_pitch_standup)
        {
            std::cout << "Current x -> Driving Wheel Angle:"
                    << x(DRIVING_WHEEL_ANGLE)
                    << '\n';
            current_setpoint(DRIVING_WHEEL_ANGLE) = x(DRIVING_WHEEL_ANGLE) + pitch_standup_shift_setpoint_drive_wheel_ang;
            reset_setpoint_pitch_standup = false;
            std::cout << "Current setpoint -> Driving Wheel Angle:"
                    << current_setpoint(DRIVING_WHEEL_ANGLE)
                    << '\n';
        }

        // Checks whether the AMPC controller has a duration, default value is set to -1 (no duration limit of control job).
        if (control_job_duration != -1) controller_counter++;

        // After an initially given duration for a pitch standup,
        // the controller gains are updated for regular operation.
        if (toggle_gains_after_pitch_standup && controller_counter == pitch_duration_switch_to_regular_operation) {
            set_gains_regular_operation();
            toggle_gains_after_pitch_standup = false;
        }

        // After an initially given duration for a flip standup,
        // the controller gains are updated for regular operation.
        if (toggle_gains_after_flip_standup && controller_counter == flip_duration_switch_to_regular_operation) {
            set_gains_regular_operation();
            toggle_gains_after_flip_standup = false;
        }

        // When the control job duration limit is reached, the respective variable is set to true.
        if (control_job_duration > 0 && controller_counter >= control_job_duration){
            control_job_done = true;
        }


        return clip(u, -u_max, u_max);
    }

    bool AmIDone() override{
        return control_job_done;
    }

    void init() override{
        last_maneuver_starting_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        controller_counter = 0;
        control_job_done = false;
        wheel_roll_ang=0;
        wheel_roll_ang_last=0;
        wheel_pitch_ang=0;
        wheel_pitch_ang_last=0;
    };

    void set_gains_pitch_standup() {
        std::cout << "gains set for pitch standup" << std::endl;
        K_roll = K_roll_pitch_standup;
        K_pitch = K_pitch_pitch_standup;
        toggle_gains_after_pitch_standup = true;
    }

    void set_gains_flip_standup() {
        std::cout << "gains set for flip standup" << std::endl;
        K_roll = K_roll_flip_standup;
        K_pitch = K_pitch_flip_standup;
        toggle_gains_after_flip_standup = true;
    }

    void set_gains_regular_operation() {
        std::cout << "gains set for regular operation" << std::endl;
        K_roll = K_roll_regular;
        K_pitch = K_pitch_regular;
    }

    void set_fallback_gains_roll_standup() {
        std::cout << "fallback gains set for reset of initial position" << std::endl;
        K_roll = K_roll_regular_fallback;
        K_pitch = K_pitch_regular_fallback;
        toggle_state_feedback_shift_setpoint_drive_wheel_pos = false;
    }

    void set_fallback_gains_pitch_standup() {
        std::cout << "fallback gains set for reset of initial position" << std::endl;
        K_roll = K_roll_pitch_standup_fallback;
        K_pitch = K_pitch_pitch_standup_fallback;
        toggle_gains_after_pitch_standup = true;
        toggle_state_feedback_shift_setpoint_drive_wheel_pos = false;
    }

    void set_setpoint_shift_neg() {
        shift_setpoint_drive_wheel_ang = -abs(shift_setpoint_drive_wheel_ang);
    }

    void set_setpoint_shift_pos() {
        shift_setpoint_drive_wheel_ang = abs(shift_setpoint_drive_wheel_ang);
    }

    void reset_setpoint_pitch_standup_pos() {
        reset_setpoint_pitch_standup = true;
        pitch_standup_shift_setpoint_drive_wheel_ang = -abs(pitch_standup_shift_setpoint);
    }

    void reset_setpoint_pitch_standup_neg() {
        reset_setpoint_pitch_standup = true;
        pitch_standup_shift_setpoint_drive_wheel_ang = abs(pitch_standup_shift_setpoint);
    }

    void set_drive_wheel_setpoint_to_current() {
        reset_setpoint = true;
    }

public:

    Eigen::Vector<float, 11> delta_theta;
    Eigen::Matrix<float,1,4> K_roll;
    Eigen::Matrix<float,1,4> K_pitch;
    Eigen::Matrix<float,1,4> K_roll_pitch_standup;
    Eigen::Matrix<float,1,4> K_pitch_pitch_standup;
    Eigen::Matrix<float,1,4> K_roll_regular;
    Eigen::Matrix<float,1,4> K_pitch_regular;
    Eigen::Matrix<float,1,4> K_roll_flip_standup;
    Eigen::Matrix<float,1,4> K_pitch_flip_standup;

    Eigen::Matrix<float,1,4> K_roll_regular_fallback;
    Eigen::Matrix<float,1,4> K_pitch_regular_fallback;
    Eigen::Matrix<float,1,4> K_roll_pitch_standup_fallback;
    Eigen::Matrix<float,1,4> K_pitch_pitch_standup_fallback;

    int64_t last_maneuver_starting_time;
    int64_t last_duration_in_millisec = 0;

    float u_max;
    float max_angles;

    enum StateIdxs : unsigned int {YAW, ROLL, PITCH, YAW_RATE, ROLL_RATE, PITCH_RATE, DRIVING_WHEEL_ANGLE, DRIVING_WHEEL_ANGULAR_VELOCITY, BALANCING_WHEEL_ANGLE, BALANCING_WHEEL_ANGULAR_VELOCITY};
    enum InputIdxs : unsigned int{TAU_DRIVING_WHEEL, TAU_BALANCING_WHEEL};
    enum ThetaIdxs : unsigned int{m_WR, m_B, I_Wxz_Ryz, I_Wy_Rx, I_Bx, I_By, I_Bz, r_W, l_WB, fric_magn, fric_slope};

    static constexpr std::array<int,4> roll_states  {ROLL, ROLL_RATE, BALANCING_WHEEL_ANGLE, BALANCING_WHEEL_ANGULAR_VELOCITY};
    static constexpr std::array<int, 1> roll_inputs {TAU_BALANCING_WHEEL};

    static constexpr std::array<int,4> pitch_states {PITCH, PITCH_RATE, DRIVING_WHEEL_ANGLE, DRIVING_WHEEL_ANGULAR_VELOCITY};
    static constexpr std::array<int, 1> pitch_inputs {TAU_DRIVING_WHEEL};

    static constexpr std::array<int,2> wheel_angles {DRIVING_WHEEL_ANGLE, BALANCING_WHEEL_ANGLE};
    static constexpr std::array<int,10> states_nn {YAW, ROLL, PITCH, YAW_RATE, ROLL_RATE, PITCH_RATE, DRIVING_WHEEL_ANGLE, BALANCING_WHEEL_ANGLE, DRIVING_WHEEL_ANGULAR_VELOCITY, BALANCING_WHEEL_ANGULAR_VELOCITY};


    float wheel_roll_ang=0;
    float wheel_roll_ang_last=0;
    float wheel_pitch_ang=0;
    float wheel_pitch_ang_last=0;
    static constexpr float wheel_roll_ang_windup=1000;
    static constexpr float wheel_pitch_ang_windup=10000;

    int controller_counter = 0;
    bool control_job_done = false;
    int control_job_duration = -1;
    bool toggle_gains_after_pitch_standup = false;
    bool toggle_gains_after_flip_standup = false;
    int pitch_duration_switch_to_regular_operation = 300;
    int flip_duration_switch_to_regular_operation = 300;

    Eigen::Vector<float, 10> current_setpoint = Eigen::Vector<float,10>::Zero();
    float shift_setpoint_drive_wheel_ang = 0;
    bool toggle_state_feedback_shift_setpoint_drive_wheel_pos = false;

    bool reset_setpoint_pitch_standup = false;
    float pitch_standup_shift_setpoint_drive_wheel_ang = 0;
    float pitch_standup_shift_setpoint = 0;

    bool reset_setpoint = false;
};

class RollStandupController : public Abstract_Controller{

    constexpr
    float deg2rad(const float deg) {
        return deg * M_PI / 180.0;
    }

public:
    int64_t last_maneuver_starting_time;
    float standup_input;
    bool is_upright;
    Eigen::Vector2f u;
    float u_max = 0.55;
    float max_input;
    float duration_first_spin;
    float duration_second_spin;

    RollStandupController(
        float max_input_,
        float duration_first_spin_,
        float duration_second_spin_):
        max_input(max_input_),
        duration_first_spin(duration_first_spin_),
        duration_second_spin(duration_second_spin_)
        {
            std::cout << "RollStandupController object created"
                    << '\n';
        }

    void init(){
        last_maneuver_starting_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        standup_input = 0.;
        is_upright = false;
        u = Eigen::Vector2f::Zero();
        std::cout << "RollStandupController object initialized"
                    << '\n';
    }

    Eigen::Vector2f update(
            Eigen::Vector<float, 10> x,
            Eigen::Vector<float, 10> setpoint = Eigen::Vector<float,10>::Zero()) override {

            int64_t duration_in_millisec = std::chrono::duration_cast
                <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count()
                 - last_maneuver_starting_time -1000;

            // CRASH DETECTION -> Detect false initial position that would lead to unsafe behaviour.
            if (duration_in_millisec < 0) {
                if (max_input > 0 && !((x(1) > deg2rad(50)) && (std::abs(x(2)) < deg2rad(10))) ||
                    max_input < 0 && !((x(1) < -deg2rad(50)) && (std::abs(x(2)) < deg2rad(10))))
                {
                    LOG_INFO_LIMIT(std::chrono::milliseconds(100), quill::get_logger(), "Wrong inital position detected! \n ----------------------- \n The process crashed! \n All threads are stopped, the experiment ends with a detected crash. \n -----------------------");
                    pthread_kill(pthread_self(), SIGINT);
                }
            }

            if (duration_in_millisec < 0) {
                standup_input = 0;
            }
            else if (duration_in_millisec >= 0 && duration_in_millisec <= duration_first_spin) {
                standup_input = -max_input * duration_in_millisec / duration_first_spin * 0.6;
            }
            else if (duration_in_millisec <= duration_first_spin + duration_second_spin) {
                standup_input = max_input;
            }
            else {
                this->is_upright = true;
                return Eigen::Vector2f::Zero();
            }

            u = {0., standup_input};

            return clip(u, -u_max, u_max);


        }


    bool AmIDone() override {
        return this->is_upright;
    }

};

class RollLayDownController : public Abstract_Controller{
public:
    int64_t last_maneuver_starting_time;
    float laydown_input;
    bool is_upright;
    Eigen::Vector2f u;
    float u_max = 0.5;
    float direction_variable = 1.;

    RollLayDownController(
        float direction_variable_):
        direction_variable(direction_variable_)
        {
            std::cout << "RollLayDownController object created"
                    << '\n';
        }

    void init(){
        last_maneuver_starting_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        laydown_input = 0.;
        is_upright = false;
        u = Eigen::Vector2f::Zero();
        std::cout << "RollLayDownController object initialized"
                    << '\n';
    }

    Eigen::Vector2f update(Eigen::Vector<float, 10> x,
            Eigen::Vector<float, 10> setpoint = Eigen::Vector<float,10>::Zero()) override {

            int64_t duration_in_millisec = std::chrono::duration_cast
                <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count()
                 - last_maneuver_starting_time;

            laydown_input = 0.;

            if (duration_in_millisec <= 70) {
                laydown_input = -0.2;
            }
            else if (duration_in_millisec > 70 && duration_in_millisec <= 140) {
                laydown_input = 0.;
            }
            else if (duration_in_millisec > 140 && duration_in_millisec <= 290) {
                laydown_input = 0.3;
            }
            else if (duration_in_millisec > 290 && duration_in_millisec <= 350) {
                laydown_input = 0.3;
            }
            else if (duration_in_millisec > 350 && duration_in_millisec <= 1500) {
                laydown_input = -0.02;
            }
            else if (duration_in_millisec > 1500 && duration_in_millisec <= 6000) { //Cooldown time
                return Eigen::Vector2f::Zero();
            }
            else {
                this->is_upright = true;
                return Eigen::Vector2f::Zero();
            }

            u = {0., laydown_input*direction_variable};

            return clip(u, -u_max, u_max);
        }


    bool AmIDone() override {
        return this->is_upright;
    }

};

class PitchStandupController : public Abstract_Controller {

    constexpr
    float deg2rad(const float deg) {
        return deg * M_PI / 180.0;
    }

public:

    int64_t last_maneuver_starting_time;
    float standup_input;
    bool is_upright;
    Eigen::Vector2f u;
    float u_max = 0.5;
    float max_input;
    float duration_first_spin;
    float duration_second_spin;
    float step_duration = 50;


    PitchStandupController(
        float max_input_,
        float duration_first_spin_,
        float duration_second_spin_,
        Config::StandUpConfig standup_config):
        max_input(max_input_),
        duration_first_spin(duration_first_spin_),
        duration_second_spin(duration_second_spin_),
        step_duration(standup_config.step_duration)
        {
            std::cout << "RollStandupController object created"
                    << '\n';
        }


    void init(){
        last_maneuver_starting_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        standup_input = 0.;
        is_upright = false;
        u = Eigen::Vector2f::Zero();
        std::cout << "PitchStandupController object initialized"
                    << '\n';
    }

    Eigen::Vector2f update(Eigen::Vector<float, 10> x,
            Eigen::Vector<float, 10> setpoint = Eigen::Vector<float,10>::Zero()) override {

            int64_t duration_in_millisec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() - last_maneuver_starting_time -1000;

            // CRASH DETECTION -> Detect false initial position that would lead to unsafe behaviour.
            if (duration_in_millisec < 0) {
                if (max_input < 0 && !((x(2) > deg2rad(50)) && (std::abs(x(1)) < deg2rad(10))) ||
                    max_input > 0 && !((x(2) < -deg2rad(50)) && (std::abs(x(1)) < deg2rad(10))))
                {
                    LOG_INFO_LIMIT(std::chrono::milliseconds(100), quill::get_logger(), "Wrong inital position detected! \n ----------------------- \n The process crashed! \n All threads are stopped, the experiment ends with a detected crash. \n -----------------------");
                    pthread_kill(pthread_self(), SIGINT);
                }
            }

            if (duration_in_millisec < 0) {
                standup_input = 0;
            }
            else if (duration_in_millisec >= 0 && duration_in_millisec <= step_duration) {
                standup_input = -max_input;
            }
            else if (duration_in_millisec > step_duration && duration_in_millisec <= 2*step_duration) {
                standup_input = -max_input*0.95;
            }
            else if (duration_in_millisec > 2*step_duration && duration_in_millisec <= 3*step_duration) {
                standup_input = -max_input*0.6;
            }
            else if (duration_in_millisec > 3*step_duration && duration_in_millisec <= 4*step_duration) {
                standup_input = -max_input*0.4;
            }
            else if (duration_in_millisec > 4*step_duration && duration_in_millisec <= 5*step_duration) {
                standup_input = -max_input*0.2;
            }
            else if (duration_in_millisec > 5*step_duration && duration_in_millisec <= duration_first_spin) {
                standup_input = -max_input*0.1;
            }

            else {
                this->is_upright = true;
                return Eigen::Vector2f::Zero();
            }

            u = {standup_input, 0.};

            return clip(u, -u_max, u_max);

        }


    bool AmIDone() override {
        return this->is_upright;
    }

};

class PitchLayDownController : public Abstract_Controller{
public:
    int64_t last_maneuver_starting_time;
    float laydown_input;
    bool is_upright;
    Eigen::Vector2f u;
    float u_max = 0.5;
    float direction_variable = 1.;
    float first_input = -0.2;
    float second_input = 0.3;
    float timestep = 70; // milliseconds

    PitchLayDownController(
        float direction_variable_,
        float first_input_,
        float second_input_,
        float timestep_):
        direction_variable(direction_variable_),
        first_input(first_input_),
        second_input(second_input_),
        timestep(timestep_)
        {
            std::cout << "PitchLayDownController object created"
                    << '\n';
        }

    void init(){
        last_maneuver_starting_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        laydown_input = 0.;
        is_upright = false;
        u = Eigen::Vector2f::Zero();
        std::cout << "PitchLayDownController object initialized"
                    << '\n';
    }

    Eigen::Vector2f update(Eigen::Vector<float, 10> x,
            Eigen::Vector<float, 10> setpoint = Eigen::Vector<float,10>::Zero()) override {

            int64_t duration_in_millisec = std::chrono::duration_cast
                <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count()
                 - last_maneuver_starting_time;

            laydown_input = 0.;

            if (duration_in_millisec <= 80) {
                laydown_input = first_input;
            }
            else if (duration_in_millisec > 80 && duration_in_millisec <= 1000) {
                laydown_input = 0.;
            }
            else if (duration_in_millisec > 1000 && duration_in_millisec <= (1000+timestep)) {
                laydown_input = second_input;
            }
            else if (duration_in_millisec > (1000+timestep) && duration_in_millisec <= 4000) {
                return Eigen::Vector2f::Zero();
            }
            else {
                this->is_upright = true;
                return Eigen::Vector2f::Zero();
            }

            u = {laydown_input*direction_variable, 0.};

            return clip(u, -u_max, u_max);
        }


    bool AmIDone() override {
        return this->is_upright;
    }

};

class FlipStandupController : public Abstract_Controller{

public:
    int64_t last_maneuver_starting_time;
    float standup_input;
    bool is_upright;
    Eigen::Vector2f u;
    float u_max = 0.5;
    float max_input;
    float flip_duration;
    float idle_time_after_flip;

    FlipStandupController(
        float max_input_,
        float flip_duration_,
        float idle_time_after_flip_):
        max_input(max_input_),
        flip_duration(flip_duration_),
        idle_time_after_flip(idle_time_after_flip_)
        {
            std::cout << "FlipStandupController object created"
                    << '\n';
        }

    void init(){
        last_maneuver_starting_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        standup_input = 0.;
        is_upright = false;
        u = Eigen::Vector2f::Zero();
        std::cout << "FlipStandupController object initialized"
                    << '\n';
    }

    Eigen::Vector2f update(Eigen::Vector<float, 10> x,
            Eigen::Vector<float, 10> setpoint = Eigen::Vector<float,10>::Zero()) override {

            int64_t duration_in_millisec = std::chrono::duration_cast
                <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count()
                 - last_maneuver_starting_time -4000;

            if (duration_in_millisec < 0) {
                standup_input = 0;
            }
            else if (duration_in_millisec >= 0 && duration_in_millisec <= flip_duration) {
                standup_input = max_input * (1-0.8*duration_in_millisec/flip_duration);
                if (duration_in_millisec >= 120) {
                    this->is_upright = true;
                }
            }
            else if (duration_in_millisec <= flip_duration + idle_time_after_flip) {
                standup_input = 0;
            }
            else {
                this->is_upright = true;
                return Eigen::Vector2f::Zero();
            }

            u = {0., standup_input};

            return clip(u, -u_max, u_max);


        }

    bool AmIDone() override {
        return this->is_upright;
    }

};



struct Controller_Container {

    RollStandupController &standup_roll_pos, &standup_roll_neg;
    RollLayDownController &laydown_roll_pos, &laydown_roll_neg;
    ParameterAdaptiveAMPC &ampc_controller ;
    PitchStandupController &standup_pitch_pos, &standup_pitch_neg;
    PitchLayDownController &laydown_pitch_pos, &laydown_pitch_neg;
    FlipStandupController &flip_standup_roll_pos, &flip_standup_roll_neg;
    int active_controller_id = -1;

public:
    Controller_Container(
        RollStandupController &standup_roll_pos_,
        RollStandupController &standup_roll_neg_,
        RollLayDownController &laydown_roll_pos_,
        RollLayDownController &laydown_roll_neg_,
        ParameterAdaptiveAMPC &ampc_controller_,
        PitchStandupController &standup_pitch_pos_,
        PitchStandupController &standup_pitch_neg_,
        PitchLayDownController &laydown_pitch_pos_,
        PitchLayDownController &laydown_pitch_neg_,
        FlipStandupController &flip_standup_roll_pos_,
        FlipStandupController &flip_standup_roll_neg_) :
        standup_roll_pos(standup_roll_pos_),
        standup_roll_neg(standup_roll_neg_),
        laydown_roll_pos(laydown_roll_pos_),
        laydown_roll_neg(laydown_roll_neg_),
        ampc_controller(ampc_controller_),
        standup_pitch_pos(standup_pitch_pos_),
        standup_pitch_neg(standup_pitch_neg_),
        laydown_pitch_pos(laydown_pitch_pos_),
        laydown_pitch_neg(laydown_pitch_neg_),
        flip_standup_roll_pos(flip_standup_roll_pos_),
        flip_standup_roll_neg(flip_standup_roll_neg_)
        {
            std::cout << "Controller_Container structure created"
                    << '\n';
        }


    Eigen::Vector2f update(Eigen::Vector<float, 10> x,
            Eigen::Vector<float, 10> setpoint = Eigen::Vector<float,10>::Zero()) {
        switch (active_controller_id) {
            case -1:
                return Eigen::Vector2f::Zero();
            case 0:
                return standup_roll_pos.update(x,setpoint);
            case 1:
                return standup_roll_neg.update(x,setpoint);
            case 2:
                return laydown_roll_pos.update(x,setpoint);
            case 3:
                return laydown_roll_neg.update(x,setpoint);
            case 4:
                return ampc_controller.update(x,setpoint);
            case 5:
                return standup_pitch_pos.update(x,setpoint);
            case 6:
                return standup_pitch_neg.update(x,setpoint);
            case 7:
                return laydown_pitch_pos.update(x,setpoint);
            case 8:
                return laydown_pitch_neg.update(x,setpoint);
            case 9:
                return flip_standup_roll_pos.update(x,setpoint);
            case 10:
                return flip_standup_roll_neg.update(x,setpoint);

        return Eigen::Vector<float,2>::Zero();
        }
    }

    bool AmIDone() {
        switch (active_controller_id) {
            case -1:
                return false;
            case 0:
                return standup_roll_pos.AmIDone();
            case 1:
                return standup_roll_neg.AmIDone();
            case 2:
                return laydown_roll_pos.AmIDone();
            case 3:
                return laydown_roll_neg.AmIDone();
            case 4:
                return ampc_controller.AmIDone();
            case 5:
                return standup_pitch_pos.AmIDone();
            case 6:
                return standup_pitch_neg.AmIDone();
            case 7:
                return laydown_pitch_pos.AmIDone();
            case 8:
                return laydown_pitch_neg.AmIDone();
            case 9:
                return flip_standup_roll_pos.AmIDone();
            case 10:
                return flip_standup_roll_neg.AmIDone();

        return 0;
        }

    }

    void init() {
        switch (active_controller_id) {
            case -1:
                std::cout << "init() called without active controller" << std::endl;
                return;
            case 0:
                std::cout << "standup_roll_pos.init() called" << std::endl;
                standup_roll_pos.init();
                ampc_controller.set_drive_wheel_setpoint_to_current();
                ampc_controller.set_setpoint_shift_pos();
                ampc_controller.u_max = 0.5;
                ampc_controller.toggle_state_feedback_shift_setpoint_drive_wheel_pos=true;
                return;
            case 1:
                std::cout << "standup_roll_neg.init() called" << std::endl;
                standup_roll_neg.init();
                ampc_controller.set_drive_wheel_setpoint_to_current();
                ampc_controller.set_setpoint_shift_neg();
                ampc_controller.u_max = 0.55;
                ampc_controller.toggle_state_feedback_shift_setpoint_drive_wheel_pos=true;
                return;
            case 2:
                std::cout << "laydown_roll_pos.init() called" << std::endl;
                laydown_roll_pos.init();
                return;
            case 3:
                std::cout << "laydown_roll_neg.init() called" << std::endl;
                laydown_roll_neg.init();
                return;
            case 4:
                std::cout << "ampc_controller.init() called" << std::endl;
                ampc_controller.init();
                return;
            case 5:
                std::cout << "standup_pitch_pos.init(x) called" << std::endl;
                ampc_controller.set_gains_pitch_standup();
                ampc_controller.reset_setpoint_pitch_standup_pos();
                ampc_controller.set_setpoint_shift_neg();
                standup_pitch_pos.init();
                return;
            case 6:
                std::cout << "standup_pitch_neg.init(x) called" << std::endl;
                ampc_controller.set_gains_pitch_standup();
                ampc_controller.reset_setpoint_pitch_standup_neg();
                ampc_controller.set_setpoint_shift_pos();
                standup_pitch_neg.init();
                return;
            case 7:
                std::cout << "laydown_pitch_pos.init() called" << std::endl;
                laydown_pitch_pos.init();
                return;
            case 8:
                std::cout << "laydown_pitch_neg.init() called" << std::endl;
                laydown_pitch_neg.init();
                return;
            case 9:
                std::cout << "flip_standup_roll_pos.init() called" << std::endl;
                ampc_controller.set_gains_flip_standup();
                flip_standup_roll_pos.init();
                return;
            case 10:
                std::cout << "flip_standup_roll_neg.init() called" << std::endl;
                ampc_controller.set_gains_flip_standup();
                flip_standup_roll_neg.init();
                return;

        return;
        }
    }
};



class ControllerSupervisor{

    constexpr
    float deg2rad(const float deg) {
        return deg * M_PI / 180.0;
    }


public:

    std::vector<int> controller_id_sequence = {};
    bool fixed_sequence_experiment_mode = false;
    int64_t flip_control_loop_update_rate,
        regular_control_loop_update_rate,
        duration_high_update_rate_after_flip_standup;
    int64_t timer_high_update_rate_after_flip_standup = 0;

    ControllerSupervisor(
        Config::StandUpConfig standup_config):
        fixed_sequence_experiment_mode(standup_config.fixed_sequence_experiment_mode),
        regular_control_loop_update_rate(standup_config.regular_control_loop_update_rate),
        flip_control_loop_update_rate(standup_config.flip_control_loop_update_rate),
        duration_high_update_rate_after_flip_standup(standup_config.duration_high_update_rate_after_flip_standup)
    {

    }

    void update(Controller_Container& cont_controllers_){
        if(cont_controllers_.AmIDone()){
            if (fixed_sequence_experiment_mode && controller_id_sequence.empty()) {
                LOG_INFO_LIMIT(std::chrono::milliseconds(100), quill::get_logger(), "Experiment has ended. \n ----------------------- \n The experiment sequence has ended. \n All threads are stopped, the experiment ends without crash. \n -----------------------");
                pthread_kill(pthread_self(), SIGINT);
            }
            std::cout << "The last controller has finished, time for a new controller"
                << '\n';
            // last controller is done, if the controller sequence is not over yet, pick next controller id from vector.
            if (!controller_id_sequence.empty()){
                cont_controllers_.active_controller_id = controller_id_sequence[0];
                controller_id_sequence.erase(controller_id_sequence.begin());
            }
            else {
                // if no controller left in sequence, disable controllers
                cont_controllers_.active_controller_id = -1;
                return;
            }
            cont_controllers_.init();
            return;
        }
        return;
    }

    void set_controller_id_sequence(std::vector<int> controller_id_sequence_){
        controller_id_sequence = {};
        for (int controller_id : controller_id_sequence_){
            controller_id_sequence.push_back(controller_id);
        }
    }

    void set_first_controller_active(Controller_Container& cont_controllers_) {
        if (controller_id_sequence.size() > 0) {
            // Set the active controller id of the controller container structure to
            // the first id in the controller sequence, increment the counter to adhere to the sequence.
            cont_controllers_.active_controller_id = controller_id_sequence[0];
            controller_id_sequence.erase(controller_id_sequence.begin());
            std::cout << "The next controller in the sequence will be:"
                << controller_id_sequence[0]
                << '\n';
            return;
        }
        else {
            // If no controller sequence is given, deactivate controller and wait for the sequence update.
            cont_controllers_.active_controller_id = -1;
        }
    }

    void set_next_controller_active(Controller_Container& cont_controllers_) {
        if (controller_id_sequence.size() > 0) {
            // Set the active controller id of the controller container structure to
            // the first id in the controller sequence, increment the counter to adhere to the sequence.
            cont_controllers_.active_controller_id = controller_id_sequence[0];
            cont_controllers_.init();
            controller_id_sequence.erase(controller_id_sequence.begin());
            return;
        }
        else {
            // If no controller sequence is given, stick to the ampc-controller.
            cont_controllers_.active_controller_id = 4;
        }
    }

    void append_sequence (std::vector<int> sequence_to_append) {
        for (int e: sequence_to_append){
            controller_id_sequence.push_back(e);
        }
    }

    int64_t update_control_loop_update_rate(Controller_Container& cont_controllers_) {
        if (cont_controllers_.active_controller_id == 9 ||
            cont_controllers_.active_controller_id == 10){
                timer_high_update_rate_after_flip_standup = duration_high_update_rate_after_flip_standup;
                return flip_control_loop_update_rate;
        }
        else if (timer_high_update_rate_after_flip_standup > 0)
        {
            std::cout << "After the flip, current control_loop_update_rate is: "
                        << flip_control_loop_update_rate
                        << '\n';
            timer_high_update_rate_after_flip_standup--;
            return flip_control_loop_update_rate;
        }
        else {
            return regular_control_loop_update_rate;
        }

    }

    void update_control_sequence(Controller_Container& cont_controllers_, Eigen::Vector<float, 10> x) {
        if (controller_id_sequence.empty()){

            if (!(fixed_sequence_experiment_mode && (std::abs(x(9)) < deg2rad(10)))){
                // If there was no fixed experiment sequence, select next controller.
                if ((x(1) > deg2rad(50)) && (std::abs(x(2)) < deg2rad(10)) && (std::abs(x(9)) < deg2rad(2))){
                    // Mini Wheelbot is lying down and has to perform a standup maneuver in positive roll direction.
                    std::vector<int> seq = {0, 4, 3};
                    append_sequence(seq);
                    set_first_controller_active(cont_controllers_);
                    cont_controllers_.init();
                    cont_controllers_.ampc_controller.set_gains_regular_operation();
                }
                else if ((x(1) < -deg2rad(50)) && (std::abs(x(2)) < deg2rad(10)) && (std::abs(x(9)) < deg2rad(2))){
                    // Mini Wheelbot is lying down and has to perform a standup maneuver in positive roll direction.
                    std::vector<int> seq = {1, 4, 2};
                    append_sequence(seq);
                    set_first_controller_active(cont_controllers_);
                    cont_controllers_.init();
                    cont_controllers_.ampc_controller.set_gains_regular_operation();
                }
                else if ((std::abs(x(1)) < deg2rad(10)) && (x(2) > deg2rad(50)) && (std::abs(x(9)) < deg2rad(0.5)) && (std::abs(x(7)) < deg2rad(2))){
                    // Mini Wheelbot is lying down and has to perform a standup maneuver in positive roll direction.
                    std::vector<int> seq = {6, 4, 8};
                    append_sequence(seq);
                    set_first_controller_active(cont_controllers_);
                    cont_controllers_.init();
                    cont_controllers_.ampc_controller.set_gains_pitch_standup();
                }
                else if ((std::abs(x(1)) < deg2rad(10)) && (x(2) < -deg2rad(50)) && (std::abs(x(9)) < deg2rad(0.5)) && (std::abs(x(7)) < deg2rad(2))){
                    // Mini Wheelbot is lying down and has to perform a standup maneuver in positive roll direction.
                    std::vector<int> seq = {5, 4, 7};
                    append_sequence(seq);
                    set_first_controller_active(cont_controllers_);
                    cont_controllers_.init();
                    cont_controllers_.ampc_controller.set_gains_pitch_standup();
                }
            }
            else if (cont_controllers_.AmIDone()) {
                LOG_INFO_LIMIT(std::chrono::milliseconds(100), quill::get_logger(), "Experiment has ended. \n ----------------------- \n The experiment sequence has ended. \n All threads are stopped, the experiment ends without crash. \n -----------------------");
                pthread_kill(pthread_self(), SIGINT);
            }
        }
    }

    void reset_initial_position(Controller_Container& cont_controllers_, Eigen::Vector<float, 10> x) {

        // Set the right laydown controller id.
        int laydown_id;
        if (cont_controllers_.active_controller_id == ControllerIdxs::ROLL_STANDUP_POS)
            {laydown_id = ControllerIdxs::ROLL_LAYDOWN_POS;}
        else if (cont_controllers_.active_controller_id == ControllerIdxs::ROLL_STANDUP_NEG)
            {laydown_id = ControllerIdxs::ROLL_LAYDOWN_NEG;}
        else if (cont_controllers_.active_controller_id == ControllerIdxs::PITCH_STANDUP_POS)
            {laydown_id = ControllerIdxs::PITCH_LAYDOWN_POS;}
        else if (cont_controllers_.active_controller_id == ControllerIdxs::PITCH_STANDUP_NEG)
            {laydown_id = ControllerIdxs::PITCH_LAYDOWN_NEG;}


        if ((x(1) > deg2rad(50)) && (std::abs(x(2)) < deg2rad(10)) && (std::abs(x(9)) < deg2rad(2))){
            // Mini Wheelbot is lying down and has to perform a standup maneuver in positive roll direction.
            std::vector<int> seq = {0, 4, laydown_id};
            controller_id_sequence = seq;
            set_first_controller_active(cont_controllers_);
            cont_controllers_.init();
            cont_controllers_.ampc_controller.set_fallback_gains_roll_standup();
        }
        else if ((x(1) < -deg2rad(50)) && (std::abs(x(2)) < deg2rad(10)) && (std::abs(x(9)) < deg2rad(2))){
            // Mini Wheelbot is lying down and has to perform a standup maneuver in negative roll direction.
            std::vector<int> seq = {1, 4, laydown_id};
            controller_id_sequence = seq;
            set_first_controller_active(cont_controllers_);
            cont_controllers_.init();
            cont_controllers_.ampc_controller.set_fallback_gains_roll_standup();
        }
        else if ((std::abs(x(1)) < deg2rad(10)) && (x(2) > deg2rad(50)) && (std::abs(x(9)) < deg2rad(0.5)) && (std::abs(x(7)) < deg2rad(2))){
            // Mini Wheelbot is lying down and has to perform a standup maneuver in negative pitch direction.
            std::vector<int> seq = {6, 4, laydown_id};
            controller_id_sequence = seq;
            set_first_controller_active(cont_controllers_);
            cont_controllers_.init();
            cont_controllers_.ampc_controller.set_fallback_gains_pitch_standup();
        }
        else if ((std::abs(x(1)) < deg2rad(10)) && (x(2) < -deg2rad(50)) && (std::abs(x(9)) < deg2rad(0.5)) && (std::abs(x(7)) < deg2rad(2))){
            // Mini Wheelbot is lying down and has to perform a standup maneuver in positive pitch direction.
            std::vector<int> seq = {5, 4, laydown_id};
            controller_id_sequence = seq;
            set_first_controller_active(cont_controllers_);
            cont_controllers_.init();
            cont_controllers_.ampc_controller.set_fallback_gains_pitch_standup();
        }
    }

    void check_initial_position_for_predefined_control_sequence(Controller_Container& cont_controllers_, Eigen::Vector<float, 10> x) {
        // if there is no predefined sequence of controllers, there is nothing to check.
        if (controller_id_sequence.empty()) {
            std::cout << "The robot has no predefined controller sequence, so there is nothing to check."
                << '\n';
            return;
        }

        if (cont_controllers_.active_controller_id == ControllerIdxs::ROLL_LAYDOWN_POS ||
            cont_controllers_.active_controller_id == ControllerIdxs::ROLL_LAYDOWN_NEG ||
            cont_controllers_.active_controller_id == ControllerIdxs::STATE_FEEDBACK ||
            cont_controllers_.active_controller_id == ControllerIdxs::PITCH_LAYDOWN_POS ||
            cont_controllers_.active_controller_id == ControllerIdxs::PITCH_LAYDOWN_NEG ||
            cont_controllers_.active_controller_id == ControllerIdxs::FLIP_STANDUP_ROLL_POS ||
            cont_controllers_.active_controller_id == ControllerIdxs::FLIP_STANDUP_ROLL_NEG)
        {
            return;
        }
        else if (cont_controllers_.active_controller_id == ControllerIdxs::ROLL_STANDUP_POS &&
            ((x(1) > deg2rad(50)) && (std::abs(x(2)) < deg2rad(10)) && (std::abs(x(9)) < deg2rad(2))))
        {
            std::cout << "The robot is in the right initial position for a positive roll stand-up!"
                << '\n';
            return;
        }
        else if (cont_controllers_.active_controller_id == ControllerIdxs::ROLL_STANDUP_NEG &&
            ((x(1) < -deg2rad(50)) && (std::abs(x(2)) < deg2rad(10)) && (std::abs(x(9)) < deg2rad(2))))
        {
            std::cout << "The robot is in the right initial position for a negative roll stand-up!"
                    << '\n';
            return;
        }
        else if (cont_controllers_.active_controller_id == ControllerIdxs::PITCH_STANDUP_NEG &&
            ((std::abs(x(1)) < deg2rad(10)) && (x(2) > deg2rad(50)) && (std::abs(x(9)) < deg2rad(0.5)) && (std::abs(x(7)) < deg2rad(2))))
        {
            std::cout << "The robot is in the right initial position for a negative pitch stand-up!"
                    << '\n';
            return;
        }
        else if (cont_controllers_.active_controller_id == ControllerIdxs::PITCH_STANDUP_POS &&
            ((std::abs(x(1)) < deg2rad(10)) && (x(2) < -deg2rad(50)) && (std::abs(x(9)) < deg2rad(0.5)) && (std::abs(x(7)) < deg2rad(2))))
        {
            std::cout << "The robot is in the right initial position for a positive pitch stand-up!"
                    << '\n';
            return;
        }
        else {
            std::cout << "Error: Wrong initial position! \n"
                << "The robot is not in the right initial position to perform the given controller sequence. \n"
                << "The controller sequence is aborted and the robot has to move to the right initial position!"
                << '\n';
            reset_initial_position(cont_controllers_, x);
        }


    }

};


using Controller = DecoupledRollPitchLqrController;

#endif
