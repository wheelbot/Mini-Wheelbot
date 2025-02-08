// MIT License

// Copyright (c) 2024 Henrik Hose

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

#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <iostream>
#include <nlohmann/json.hpp>
#include <array>

using json = nlohmann::json;

namespace Config {
    struct AmpcConfig {
        float delta_m_WR;
        float delta_m_B;
        float delta_I_Wxz_Ryz;
        float delta_I_Wy_Rx;
        float delta_I_Bx;
        float delta_I_By;
        float delta_I_Bz;
        float delta_r_W;
        float delta_l_WB;
        float delta_fric_magn;
        float delta_fric_slope;

        std::array<float, 11>
        toArray() const
        {
        return {
            delta_m_WR,
            delta_m_B,
            delta_I_Wxz_Ryz,
            delta_I_Wy_Rx,
            delta_I_Bx,
            delta_I_By,
            delta_I_Bz,
            delta_r_W,
            delta_l_WB,
            delta_fric_magn,
            delta_fric_slope
            };
        }
    };

    struct BalancingConfig {
        std::array<float, 4> K_roll;
        std::array<float, 4> K_pitch;
    };

    struct StandUpConfig {
        std::array<float, 3> inputs_standup_rp;
        std::array<float, 3> inputs_standup_rn;
        float inputs_laydown_rp;
        float inputs_laydown_rn;
        std::array<float, 3> inputs_pitch_standup_rp;
        std::array<float, 3> inputs_pitch_standup_rn;
        std::array<float, 4> inputs_pitch_laydown_rp;
        std::array<float, 4> inputs_pitch_laydown_rn;
        std::array<float, 3> inputs_flip_standup_roll_pos;
        std::array<float, 3> inputs_flip_standup_roll_neg;
        std::vector<int> controller_id_sequence;
        std::vector<int> controller_id_sequence_roll_opt;
        std::vector<int> controller_id_sequence_pitch_opt;
        float steady_state_control_job_duration;
        std::array<float, 4> K_roll_pitch_standup_fallback;
        std::array<float, 4> K_pitch_pitch_standup_fallback;
        std::array<float, 4> K_roll_pitch_standup;
        std::array<float, 4> K_pitch_pitch_standup;
        std::array<float, 4> K_roll_flip_standup;
        std::array<float, 4> K_pitch_flip_standup;
        int64_t regular_control_loop_update_rate;
        int64_t flip_control_loop_update_rate;
        int64_t duration_high_update_rate_after_flip_standup;
        float detection_angle_pitch;
        float detection_angle_roll;
        float flip_detection_angle_pitch;
        float flip_detection_angle_roll;
        float step_duration;
        float pitch_duration_switch_to_regular_operation;
        float flip_duration_switch_to_regular_operation;
        float shift_setpoint_drive_wheel_ang;
        bool toggle_state_feedback_shift_setpoint_drive_wheel_pos;
        float pitch_standup_shift_setpoint;
        bool fixed_sequence_experiment_mode;
    };

    struct MainConfig {
        AmpcConfig ampc_config;
        BalancingConfig balancing_config;
        StandUpConfig standup_config;
    };

    void to_json(json& j, const AmpcConfig& c) {
        j = json{
            {"delta_m_WR", c.delta_m_WR},
            {"delta_m_B", c.delta_m_B},
            {"delta_I_Wxz_Ryz", c.delta_I_Wxz_Ryz},
            {"delta_I_Wy_Rx", c.delta_I_Wy_Rx},
            {"delta_I_Bx", c.delta_I_Bx},
            {"delta_I_By", c.delta_I_By},
            {"delta_I_Bz", c.delta_I_Bz},
            {"delta_r_W", c.delta_r_W},
            {"delta_l_WB", c.delta_l_WB},
            {"delta_fric_magn", c.delta_fric_magn},
            {"delta_fric_slope", c.delta_fric_slope}
        };
    }

    void from_json(const json& j, AmpcConfig& c) {
        j.at("delta_m_WR").get_to(c.delta_m_WR);
        j.at("delta_m_B").get_to(c.delta_m_B);
        j.at("delta_I_Wxz_Ryz").get_to(c.delta_I_Wxz_Ryz);
        j.at("delta_I_Wy_Rx").get_to(c.delta_I_Wy_Rx);
        j.at("delta_I_Bx").get_to(c.delta_I_Bx);
        j.at("delta_I_By").get_to(c.delta_I_By);
        j.at("delta_I_Bz").get_to(c.delta_I_Bz);
        j.at("delta_r_W").get_to(c.delta_r_W);
        j.at("delta_l_WB").get_to(c.delta_l_WB);
        j.at("delta_fric_magn").get_to(c.delta_fric_magn);
        j.at("delta_fric_slope").get_to(c.delta_fric_slope);
    }

    void to_json(json& j, const BalancingConfig& c) {
        j = json{
            {"Kroll", c.K_roll},
            {"Kpitch", c.K_pitch}
        };
    }

    void from_json(const json& j, BalancingConfig& c) {
        j.at("Kroll").get_to(c.K_roll);
        j.at("Kpitch").get_to(c.K_pitch);
    }

    void to_json(json& j, const StandUpConfig& c) {
        j = json{
            {"inputs_standup_rp", c.inputs_standup_rp},
            {"inputs_standup_rn", c.inputs_standup_rn},
            {"inputs_laydown_rp", c.inputs_laydown_rp},
            {"inputs_laydown_rn", c.inputs_laydown_rn},
            {"inputs_pitch_standup_rp", c.inputs_pitch_standup_rp},
            {"inputs_pitch_standup_rn", c.inputs_pitch_standup_rn},
            {"inputs_pitch_laydown_rp", c.inputs_pitch_laydown_rp},
            {"inputs_pitch_laydown_rn", c.inputs_pitch_laydown_rn},
            {"inputs_flip_standup_roll_pos", c.inputs_flip_standup_roll_pos},
            {"inputs_flip_standup_roll_neg", c.inputs_flip_standup_roll_neg},
            {"controller_id_sequence", c.controller_id_sequence},
            {"controller_id_sequence_roll_opt", c.controller_id_sequence_roll_opt},
            {"controller_id_sequence_pitch_opt", c.controller_id_sequence_pitch_opt},
            {"steady_state_control_job_duration", c.steady_state_control_job_duration},
            {"K_roll_pitch_standup_fallback", c.K_roll_pitch_standup_fallback},
            {"K_pitch_pitch_standup_fallback", c.K_pitch_pitch_standup_fallback},
            {"K_roll_pitch_standup", c.K_roll_pitch_standup},
            {"K_pitch_pitch_standup", c.K_pitch_pitch_standup},
            {"K_roll_flip_standup", c.K_roll_flip_standup},
            {"K_pitch_flip_standup", c.K_pitch_flip_standup},
            {"regular_control_loop_update_rate", c.regular_control_loop_update_rate},
            {"flip_control_loop_update_rate", c.flip_control_loop_update_rate},
            {"duration_high_update_rate_after_flip_standup", c.duration_high_update_rate_after_flip_standup},
            {"detection_angle_pitch", c.detection_angle_pitch},
            {"detection_angle_roll", c.detection_angle_roll},
            {"flip_detection_angle_pitch", c.flip_detection_angle_pitch},
            {"flip_detection_angle_roll", c.flip_detection_angle_roll},
            {"step_duration", c.step_duration},
            {"pitch_duration_switch_to_regular_operation", c.pitch_duration_switch_to_regular_operation},
            {"flip_duration_switch_to_regular_operation", c.flip_duration_switch_to_regular_operation},
            {"shift_setpoint_drive_wheel_ang", c.shift_setpoint_drive_wheel_ang},
            {"toggle_state_feedback_shift_setpoint_drive_wheel_pos", c.toggle_state_feedback_shift_setpoint_drive_wheel_pos},
            {"pitch_standup_shift_setpoint", c.pitch_standup_shift_setpoint},
            {"fixed_sequence_experiment_mode", c.fixed_sequence_experiment_mode}
        };
    }

    void from_json(const json& j, StandUpConfig& c) {
        j.at("inputs_standup_rp").get_to(c.inputs_standup_rp);
        j.at("inputs_standup_rn").get_to(c.inputs_standup_rn);
        j.at("inputs_laydown_rp").get_to(c.inputs_laydown_rp);
        j.at("inputs_laydown_rn").get_to(c.inputs_laydown_rn);
        j.at("inputs_pitch_standup_rp").get_to(c.inputs_pitch_standup_rp);
        j.at("inputs_pitch_standup_rn").get_to(c.inputs_pitch_standup_rn);
        j.at("inputs_pitch_laydown_rp").get_to(c.inputs_pitch_laydown_rp);
        j.at("inputs_pitch_laydown_rn").get_to(c.inputs_pitch_laydown_rn);
        j.at("inputs_flip_standup_roll_pos").get_to(c.inputs_flip_standup_roll_pos);
        j.at("inputs_flip_standup_roll_neg").get_to(c.inputs_flip_standup_roll_neg);
        j.at("controller_id_sequence").get_to(c.controller_id_sequence);
        j.at("controller_id_sequence_roll_opt").get_to(c.controller_id_sequence_roll_opt);
        j.at("controller_id_sequence_pitch_opt").get_to(c.controller_id_sequence_pitch_opt);
        j.at("steady_state_control_job_duration").get_to(c.steady_state_control_job_duration);
        j.at("K_roll_pitch_standup_fallback").get_to(c.K_roll_pitch_standup_fallback);
        j.at("K_pitch_pitch_standup_fallback").get_to(c.K_pitch_pitch_standup_fallback);
        j.at("K_roll_pitch_standup").get_to(c.K_roll_pitch_standup);
        j.at("K_pitch_pitch_standup").get_to(c.K_pitch_pitch_standup);
        j.at("K_roll_flip_standup").get_to(c.K_roll_flip_standup);
        j.at("K_pitch_flip_standup").get_to(c.K_pitch_flip_standup);
        j.at("regular_control_loop_update_rate").get_to(c.regular_control_loop_update_rate);
        j.at("flip_control_loop_update_rate").get_to(c.flip_control_loop_update_rate);
        j.at("duration_high_update_rate_after_flip_standup").get_to(c.duration_high_update_rate_after_flip_standup);
        j.at("detection_angle_pitch").get_to(c.detection_angle_pitch);
        j.at("detection_angle_roll").get_to(c.detection_angle_roll);
        j.at("flip_detection_angle_pitch").get_to(c.flip_detection_angle_pitch);
        j.at("flip_detection_angle_roll").get_to(c.flip_detection_angle_roll);
        j.at("step_duration").get_to(c.step_duration);
        j.at("pitch_duration_switch_to_regular_operation").get_to(c.pitch_duration_switch_to_regular_operation);
        j.at("flip_duration_switch_to_regular_operation").get_to(c.flip_duration_switch_to_regular_operation);
        j.at("shift_setpoint_drive_wheel_ang").get_to(c.shift_setpoint_drive_wheel_ang);
        j.at("toggle_state_feedback_shift_setpoint_drive_wheel_pos").get_to(c.toggle_state_feedback_shift_setpoint_drive_wheel_pos);
        j.at("pitch_standup_shift_setpoint").get_to(c.pitch_standup_shift_setpoint);
        j.at("fixed_sequence_experiment_mode").get_to(c.fixed_sequence_experiment_mode);
    }

    void to_json(json& j, const MainConfig& c) {
        j = json{
            {"ampc_config", c.ampc_config},
            {"balancing_config", c.balancing_config},
            {"standup_config", c.standup_config}
        };
    }

    void from_json(const json& j, MainConfig& c) {
        j.at("ampc_config").get_to(c.ampc_config);
        j.at("balancing_config").get_to(c.balancing_config);
        j.at("standup_config").get_to(c.standup_config);
    }
} // namespace Config

#endif // CONFIG_HPP
