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

#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <memory>
#include <atomic>
#include <fstream>

#include <poll.h>

#include "wheelbot/accel.hpp"
#include "wheelbot/can.hpp"
#include "wheelbot/config.hpp"
#include "wheelbot/controller.hpp"
#include "wheelbot/constants.hpp"
#include "wheelbot/datalogger.hpp"
#include "wheelbot/estimator.hpp"
#include "wheelbot/odometry.hpp"
#include "wheelbot/gyro.hpp"
#include "wheelbot/uprightdetection.hpp"
#include "wheelbot/motorestimator.hpp"
#include "wheelbot/inputserver.hpp"

#include <nlohmann/json.hpp>
#include <argparse/argparse.hpp>
#include "quill/Quill.h"


class ControllerThread : public cactus_rt::CyclicThread {

public:
    ControllerThread(
        std::shared_ptr<MotorInterfaceThread> motor,
        const std::array<std::shared_ptr<GyroThread>,4>& gyroscopes,
        const std::array<std::shared_ptr<AccelThread>,4>& accelerometer,
        std::shared_ptr<DataLoggerThread> data_logger,
        std::shared_ptr<InputServerThread> input_server,
        Config::MainConfig config
         )
        : CyclicThread("ControllerThread", MakeRealTimeThreadConfig()),
            motor_(motor),
            gyroscopes_(gyroscopes),
            accelerometer_(accelerometer),
            data_logger_(data_logger),
            input_server_(input_server),
            main_config_(config),
            controller_(config.balancing_config, config.ampc_config, config.standup_config),
            inputs_standup_rp(config.standup_config.inputs_standup_rp),
            inputs_standup_rn(config.standup_config.inputs_standup_rn),
            inputs_laydown_rp(config.standup_config.inputs_laydown_rp),
            inputs_laydown_rn(config.standup_config.inputs_laydown_rn),
            inputs_pitch_standup_rp(config.standup_config.inputs_pitch_standup_rp),
            inputs_pitch_standup_rn(config.standup_config.inputs_pitch_standup_rn),
            controller_id_sequence(config.standup_config.controller_id_sequence),
            steady_state_control_job_duration(config.standup_config.steady_state_control_job_duration),
            regular_control_loop_update_rate(config.standup_config.regular_control_loop_update_rate),
            inputs_pitch_laydown_rp(config.standup_config.inputs_pitch_laydown_rp),
            inputs_pitch_laydown_rn(config.standup_config.inputs_pitch_laydown_rn),
            inputs_flip_standup_roll_pos(config.standup_config.inputs_flip_standup_roll_pos),
            inputs_flip_standup_roll_neg(config.standup_config.inputs_flip_standup_roll_neg),
            setpoint_(Eigen::Vector<float, 10>::Zero()),
            odometry_(Estimator<>::r)
    {
        LOG_INFO_LIMIT(std::chrono::milliseconds(100), Logger(), "Estimator X1 = {}", estimator_.X1);

        // Initialize controllers with parameters from config
        roll_standup_rp = RollStandupController(inputs_standup_rp[0], inputs_standup_rp[1], inputs_standup_rp[2]);
        roll_standup_rn = RollStandupController(inputs_standup_rn[0], inputs_standup_rn[1], inputs_standup_rn[2]);
        roll_laydown_rp = RollLayDownController(inputs_laydown_rp);
        roll_laydown_rn = RollLayDownController(inputs_laydown_rn);
        pitch_standup_rp = PitchStandupController(inputs_pitch_standup_rp[0], inputs_pitch_standup_rp[1], inputs_pitch_standup_rp[2], config.standup_config);
        pitch_standup_rn = PitchStandupController(inputs_pitch_standup_rn[0], inputs_pitch_standup_rn[1], inputs_pitch_standup_rn[2], config.standup_config);
        pitch_laydown_rp = PitchLayDownController(inputs_pitch_laydown_rp[0], inputs_pitch_laydown_rp[1], inputs_pitch_laydown_rp[2], inputs_pitch_laydown_rp[3]);
        pitch_laydown_rn = PitchLayDownController(inputs_pitch_laydown_rn[0], inputs_pitch_laydown_rn[1], inputs_pitch_laydown_rn[2], inputs_pitch_laydown_rn[3]);
        flip_standup_rp = FlipStandupController(inputs_flip_standup_roll_pos[0], inputs_flip_standup_roll_pos[1], inputs_flip_standup_roll_pos[2]);
        flip_standup_rn = FlipStandupController(inputs_flip_standup_roll_neg[0], inputs_flip_standup_roll_neg[1], inputs_flip_standup_roll_neg[2]);

        // Set control job duration for ampc-controller
        ampc_controller.control_job_duration = steady_state_control_job_duration;

        // Initialize control loop update rate for regular operation
        control_loop_update_rate = regular_control_loop_update_rate;

        // Set controller sequence, activate the first controller and initialize it
        controller_supervisor.set_controller_id_sequence(controller_id_sequence);

        if (!controller_id_sequence.empty()){
            controller_supervisor.set_first_controller_active(controller_container);
            controller_container.init();
        }

    }

    int64_t GetLoopCounter() const {
        return loop_counter_;
    }

    bool Loop(int64_t ellapsed_ns) noexcept {
        if(!starting_time.has_value()){
            starting_time=cactus_rt::NowNs();
            data_.time = 0;
        }
        else{
            data_.time = cactus_rt::NowNs() - starting_time.value();
        }
        if(!last_motor_times_.has_value()){
            auto time_now = cactus_rt::NowNs();
            last_motor_times_ = std::array<long long int, 2>{time_now, time_now};
        }

        motor_->motor.send_sync();

        Eigen::Matrix<float, 3, 4> m;
        for (size_t i=0; i<4; i++){
            m(Eigen::all, i) = gyroscopes_.at(i)->gyro.get_latest_converted_measurement();
        }

        Eigen::Matrix<float, 3, 4> a;
        for (size_t i=0; i<4; i++){
            a(Eigen::all, i) = accelerometer_.at(i)->accel.get_latest_filtered_converted_measurement();
        }

        for (auto& m : motorestimators_){
            m.predict();
        }

        while(true){
            auto res = motor_->motor.try_get_motor_state();
            if (res.has_value()){
                auto& m = motorestimators_.at(res.value().motor_id-1);
                m.update(res.value().angle);
                last_motor_times_.value().at(res.value().motor_id-1) = res.value().sync_timestamp;
            }
            else{
                break;
            }
        }

        Eigen::Matrix<float, 3,2> s = Eigen::Matrix<float, 3,2>::Zero();
        for (size_t i = 0; i < 2; i++){
            auto & m = motorestimators_.at(i);
            const auto time_now = cactus_rt::NowNs();
            auto delay = (time_now-last_motor_times_.value().at(i))/1'000'000'000.0;
            if (delay > 0.02) delay = 0;
            s(0,i) = m.get_propagated_meas_angle( delay );
            s(1,i) = m.get_propagated_velocity( delay );
            s(2,i) = m.get_acceleration();
        }

        auto x = estimator_.update(m, a, s);
        auto [odom_px, odom_py, odom_vel, odom_yaw] = odometry_.update(x(0), x(6), x(7));
        const auto is_upright = ud_.detect_latched(x);
        const auto is_in_operating_range = ud_.detect_operating_range(x);
        const auto is_in_operating_range_after_flip = ud_.detect_flip_operating_range(x);

        auto BOEXPERIMENT = false;
        if(BOEXPERIMENT){
            setpoint_counter_ += is_upright;
            if (setpoint_counter_ > 5000)   setpoint_(0) = 0.5f*std::numbers::pi_v<float>;
            if (setpoint_counter_ > 10000)  setpoint_(0) = 0;
            if (setpoint_counter_ > 15000)  setpoint_(0) = -0.5f*std::numbers::pi_v<float>;
            if (setpoint_counter_ > 20000)  setpoint_(0) = 0;
            if (setpoint_counter_ > 25000)  terminate_ = true;
        }
        else{
            input_server_->tcp_server_.set_odometry_state(WheelbotOdometryState{odom_px, odom_py, odom_vel, odom_yaw});
            auto delta_setpoint = input_server_->tcp_server_.retrieve_new_command().value_or(WheelbotInputCommand{0,0});
            setpoint_(0) = setpoint_(0) + delta_setpoint.yaw_delta;
            setpoint_(6) = setpoint_(6) + delta_setpoint.drive_delta;
        }

        // Update control_loop_update_rate if necessary.
        control_loop_update_rate = controller_supervisor.update_control_loop_update_rate(controller_container);

        if (controller_container.active_controller_id == -1 || loop_counter_ % control_loop_update_rate == 0) {
            controller_supervisor.update_control_sequence(controller_container, x);
        }

        // Check whether the robot is in the right initial position for the predefined stand-up control sequence.
        if(loop_counter_ == 0){
            controller_supervisor.check_initial_position_for_predefined_control_sequence(controller_container, x);
        }


        if (loop_counter_ % control_loop_update_rate == 0)
        {
            if (controller_container.active_controller_id != 9 &&   // For flip stand-up, there is a different supervision rule.
                controller_container.active_controller_id != 10 ) {
                    controller_supervisor.update(controller_container);
            }

            auto u = controller_container.update(x, setpoint_);
            motor_->motor.send_motor_command(u);
            data_.tau_DR_command = u;
            std::cout << "current input:"
                    << u
                    << '\n';
        }

        data_.gyro0 = m(Eigen::all,0);
        data_.gyro1 = m(Eigen::all,1);
        data_.gyro2 = m(Eigen::all,2);
        data_.gyro3 = m(Eigen::all,3);
        data_.accel0 = a(Eigen::all,0);
        data_.accel1 = a(Eigen::all,1);
        data_.accel2 = a(Eigen::all,2);
        data_.accel3 = a(Eigen::all,3);
        data_.q_yrp = x({0,1,2});
        data_.dq_yrp = x({3,4,5});
        data_.q_DR = s(0, Eigen::all);
        data_.dq_DR = s(1, Eigen::all);
        data_.ddq_DR = s(2, Eigen::all);
        data_.setpoint_yrp = setpoint_.head<3>();
        data_logger_->data_logger_.emplace_data(data_);

        loop_counter_++;

        // Catching the robot after the regular roll or pitch standup maneuvers
        if (is_in_operating_range && (
            controller_container.active_controller_id == 0 ||   // standup_roll_pos
            controller_container.active_controller_id == 1 ||   // standup_roll_neg
            controller_container.active_controller_id == 5 ||   // standup_pitch_pos
            controller_container.active_controller_id == 6 )    // standup_pitch_neg
            ) {
            std::cout << "The robot is in a position where the lqr can take over!"
                 << '\n';
            controller_supervisor.set_next_controller_active(controller_container);
        }

        // Catching the robot after the flip standup maneuvers
        if (controller_container.active_controller_id == 9 ||      // standup_flip_roll_pos
            controller_container.active_controller_id == 10 ) {    // standup_flip_pitch_neg

            if (is_in_operating_range_after_flip && controller_container.AmIDone()) {
                std::cout << "After the (hopefully) successful flip stand-up the robot is in a position where the lqr can take over!"
                    << '\n';
                controller_supervisor.set_next_controller_active(controller_container);
            }
        }

        // Crash detection

        if (controller_container.active_controller_id == 4) {
            if ((std::abs(x(1)) > 52 * M_PI / 180.0) || (std::abs(x(2)) > 56 * M_PI / 180.0)) {
                crash_detected = true;
                crash_counter_for_gentle_shutdown++;
                // After the detection of a crash, wait 4 seconds without motor input for the wheels to spin down.
                if (crash_counter_for_gentle_shutdown >= 0) {
                    LOG_INFO_LIMIT(std::chrono::milliseconds(100), Logger(), "Crash detected! \n ----------------------- \n The process crashed! \n All threads are stopped, the experiment ends with a detected crash. \n -----------------------");
                    pthread_kill(pthread_self(), SIGINT);
                }
            }
        }

        return false;
    }

private:
    std::shared_ptr<MotorInterfaceThread> motor_;
    std::array<std::shared_ptr<GyroThread>,4> gyroscopes_;
    std::array<std::shared_ptr<AccelThread>,4> accelerometer_;
    std::shared_ptr<DataLoggerThread> data_logger_;
    std::shared_ptr<InputServerThread> input_server_;
    const Config::MainConfig main_config_;
    size_t                      max_loop_counter_;
    size_t                      setpoint_counter_{0};
    size_t                      loop_counter_{0};

    static cactus_rt::CyclicThreadConfig MakeRealTimeThreadConfig() {
        cactus_rt::CyclicThreadConfig config;
        config.period_ns = 1'000'000;
        config.cpu_affinity = std::vector<size_t>{3};
        config.SetFifoScheduler(99);

        return config;
    }

    Estimator<> estimator_;
    Odometry odometry_;
    std::array<MotorEstimator, 2> motorestimators_;
    std::optional<std::array<long long int, 2>> last_motor_times_;
    ParameterAdaptiveAMPC controller_;
    Eigen::Vector<float, 10> setpoint_;
    MotorInterface<> motor_interface_;
    WheelbotStateData::Data data_;
    std::optional<uint64_t>   starting_time;

    UprightDetector ud_ = UprightDetector(main_config_.standup_config);

    bool terminate_ = false;

public:

    // Initialize all controllers with arbitrary values
    RollStandupController roll_standup_rp = RollStandupController(0,0,0);
    RollStandupController roll_standup_rn = RollStandupController(0,0,0);
    RollLayDownController roll_laydown_rp = RollLayDownController(1.0);
    RollLayDownController roll_laydown_rn = RollLayDownController(1.0);
    ParameterAdaptiveAMPC ampc_controller_ = ParameterAdaptiveAMPC(main_config_.balancing_config, main_config_.ampc_config, main_config_.standup_config);
    PitchStandupController pitch_standup_rp = PitchStandupController(0,0,0,main_config_.standup_config);
    PitchStandupController pitch_standup_rn = PitchStandupController(0,0,0,main_config_.standup_config);
    PitchLayDownController pitch_laydown_rp = PitchLayDownController(0,0,0,0);
    PitchLayDownController pitch_laydown_rn = PitchLayDownController(0,0,0,0);
    FlipStandupController flip_standup_rp = FlipStandupController(0,0,0);
    FlipStandupController flip_standup_rn = FlipStandupController(0,0,0);

    // Set pointers to controller objects
    RollStandupController& standup_roll_positive = roll_standup_rp;
    RollStandupController& standup_roll_negative = roll_standup_rn;
    ParameterAdaptiveAMPC& ampc_controller = ampc_controller_;
    RollLayDownController& laydown_roll_pos = roll_laydown_rp;
    RollLayDownController& laydown_roll_neg = roll_laydown_rn;
    PitchStandupController& standup_pitch_positive = pitch_standup_rp;
    PitchStandupController& standup_pitch_negative = pitch_standup_rn;
    PitchLayDownController& laydown_pitch_pos = pitch_laydown_rp;
    PitchLayDownController& laydown_pitch_neg = pitch_laydown_rn;
    FlipStandupController& flip_standup_roll_positive = flip_standup_rp;
    FlipStandupController& flip_standup_roll_negative = flip_standup_rn;

    // Initialize controller supervisor
    ControllerSupervisor controller_supervisor = ControllerSupervisor(main_config_.standup_config);

    // Initialize controller container
    Controller_Container controller_container =
        Controller_Container(
            standup_roll_positive, standup_roll_negative,
            laydown_roll_pos, laydown_roll_neg,
            ampc_controller,
            standup_pitch_positive, standup_pitch_negative,
            laydown_pitch_pos, laydown_pitch_neg,
            flip_standup_roll_positive, flip_standup_roll_negative);

    // Define config parameters to fill with data from config
    std::array<float, 3> inputs_standup_rp, inputs_standup_rn,
            inputs_pitch_standup_rp, inputs_pitch_standup_rn,
            inputs_flip_standup_roll_pos, inputs_flip_standup_roll_neg;

    std::array<float, 4> inputs_pitch_laydown_rp, inputs_pitch_laydown_rn;

    std::vector<int> controller_id_sequence;
    float inputs_laydown_rp, inputs_laydown_rn;
    int steady_state_control_job_duration;

    int64_t control_loop_update_rate, regular_control_loop_update_rate;

    bool crash_detected = false;
    int crash_counter_for_gentle_shutdown = 0;

};


int main(int argc, char * argv[])
{
    argparse::ArgumentParser program("Main");
    program.add_argument("-c", "--config")
        .help("path to main configuration yaml file");
    program.add_argument("-o", "--output")
        .help("path to output log csv file");

    try {
        program.parse_args(argc, argv);
    } catch (const std::exception &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return 1;
    }

    cactus_rt::App app;
    auto logger = quill::get_logger();
    LOG_INFO(logger, "Welcome to Wheelbot control app");

    #ifdef HARDWARE_BUILD
    std::string configfile = "/wheelbot-lib/config/default.json";
    #else
    std::string configfile = "../config/default.json";
    #endif
    if (program.is_used("-c")) {
        configfile = program.get("-c");
        LOG_INFO(logger, "Using user provided configfile {}", configfile);
    }
    else {
        LOG_INFO(logger, "Using default configfile {}", configfile);
    }

    std::ifstream main_config_file(configfile);
    nlohmann::json main_config_data = nlohmann::json::parse(main_config_file);
    auto main_config = main_config_data.get<Config::MainConfig>();

    #ifdef HARDWARE_BUILD
    auto loggingfile = create_filename_with_date("/wheelbot-lib/log");
    #else
    auto loggingfile = create_filename_with_date("../log");
    #endif
    if (program.is_used("-o")) {
        loggingfile = program.get("-o");
        LOG_INFO(logger, "Using user provided logfile {}", loggingfile);
    }
    else{
        LOG_INFO(logger, "Using default logfile {}", loggingfile);
    }

    auto logger_thread = std::make_shared<DataLoggerThread>(loggingfile);
    auto input_thread = std::make_shared<InputServerThread>();
    std::array gyro_threads {
        std::make_shared<GyroThread>("gyro0", R01),
        std::make_shared<GyroThread>("gyro1", R01),
        std::make_shared<GyroThread>("gyro2", R23),
        std::make_shared<GyroThread>("gyro3", R23)
        };

    std::array acc_threads {
        std::make_shared<AccelThread>("acc0", R01),
        std::make_shared<AccelThread>("acc1", R01),
        std::make_shared<AccelThread>("acc2", R23),
        std::make_shared<AccelThread>("acc3", R23)
        };

    auto motor_thread = std::make_shared<MotorInterfaceThread>("motor");
    auto controller_thread = std::make_shared<ControllerThread>(
        motor_thread, gyro_threads, acc_threads, logger_thread, input_thread, main_config);

    app.RegisterThread(motor_thread);
    for ( const auto& t : gyro_threads ){
        app.RegisterThread(t);
    }
    for ( const auto& t : acc_threads ){
        app.RegisterThread(t);
    }
    app.RegisterThread(logger_thread);
    app.RegisterThread(input_thread);
    app.RegisterThread(controller_thread);

    cactus_rt::SetUpTerminationSignalHandler();
    LOG_INFO(logger,"Testing RT loop until CTRL+C\n.");

    app.Start();

    // This function blocks until SIGINT or SIGTERM are received.
    cactus_rt::WaitForAndHandleTerminationSignal();

    app.RequestStop();
    app.Join();

    LOG_INFO(logger, "CONTROL Number of loops executed: {}", controller_thread->GetLoopCounter());
    LOG_INFO(logger, "GYRO Number of loops executed:    {}, {}, {}, {}",
        gyro_threads.at(0)->GetLoopCounter(),
        gyro_threads.at(1)->GetLoopCounter(),
        gyro_threads.at(2)->GetLoopCounter(),
        gyro_threads.at(3)->GetLoopCounter());
    LOG_INFO(logger, "ACCEL Number of loops executed:   {}, {}, {}, {}",
        acc_threads.at(0)->GetLoopCounter(),
        acc_threads.at(1)->GetLoopCounter(),
        acc_threads.at(2)->GetLoopCounter(),
        acc_threads.at(3)->GetLoopCounter());

    return 0;

}
