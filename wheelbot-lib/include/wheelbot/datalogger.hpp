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

#ifndef DATALOGGER_HPP
#define DATALOGGER_HPP

#include <iostream>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <unordered_map>

#include <eigen3/Eigen/Dense>
#include <cactus_rt/rt.h>

#include <quill/Quill.h>

template <typename T>
concept NumericType = std::integral<T> || std::floating_point<T>;


namespace WheelbotStateData{
    struct Data {
        unsigned long long int          time;
        Eigen::Vector3f                 gyro0;
        Eigen::Vector3f                 gyro1;
        Eigen::Vector3f                 gyro2;
        Eigen::Vector3f                 gyro3;
        Eigen::Vector3f                 accel0;
        Eigen::Vector3f                 accel1;
        Eigen::Vector3f                 accel2;
        Eigen::Vector3f                 accel3;
        Eigen::Vector3f                 q_yrp;
        Eigen::Vector3f                 dq_yrp;
        Eigen::Vector2f                 q_DR;
        Eigen::Vector2f                 dq_DR;
        Eigen::Vector2f                 ddq_DR;
        Eigen::Vector2f                 tau_DR_command;
        Eigen::Vector3f                 setpoint_yrp;
    };

    static const std::vector<std::string> Names {
            "time",
            "gyro0",
            "gyro1",
            "gyro2",
            "gyro3",
            "accel0",
            "accel1",
            "accel2",
            "accel3",
            "q_yrp",
            "dq_yrp",
            "q_DR",
            "dq_DR",
            "ddq_DR",
            "tau_DR_command",
            "setpoint_yrp",
    };

    static const std::vector<std::string> Headings {
        "_time",
        "/gyro0/x,/gyro0/y,/gyro0/z",
        "/gyro1/x,/gyro1/y,/gyro1/z",
        "/gyro2/x,/gyro2/y,/gyro2/z",
        "/gyro3/x,/gyro3/y,/gyro3/z",
        "/accel0/x,/accel0/y,/accel0/z",
        "/accel1/x,/accel1/y,/accel1/z",
        "/accel2/x,/accel2/y,/accel2/z",
        "/accel3/x,/accel3/y,/accel3/z",
        "/q_yrp/yaw,/q_yrp/roll,/q_yrp/pitch",
        "/dq_yrp/yaw_vel,/dq_yrp/roll_vel,/dq_yrp/pitch_vel",
        "/q_DR/drive_wheel,/q_DR/reaction_wheel",
        "/dq_DR/drive_wheel,/dq_DR/reaction_wheel",
        "/ddq_DR/drive_wheel,/ddq_DR/reaction_wheel",
        "/tau_DR_command/drive_wheel,/tau_DR_command/reaction_wheel",
        "/setpoint_yrp/yaw", "/setpoint_yrp/roll", "/setpoint_yrp/pitch"
    };

};

std::string create_filename_with_date(const std::string& path = "./", const std::string& filename = "experiment"){
    // Get the current system time
    auto currentTime = std::chrono::system_clock::now();

    // Convert the time to a string in a human-readable format
    std::time_t time = std::chrono::system_clock::to_time_t(currentTime);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d_%H-%M-%S");

    // Create the file_name by combining the variable name and timestamp
    std::string full_file_name = path + "/" + filename + "_" + ss.str() + ".csv";

    return full_file_name;
}

class DataLogger{
    size_t lines_written_ = 0;
    std::string file_path_;
    quill::Logger* logger_;
    moodycamel::ReaderWriterQueue<WheelbotStateData::Data> queue_;
    std::ofstream file_;

public:
    DataLogger(
        const std::string& file_path,
        const std::vector<std::string>& file_names = std::vector<std::string>()):
        logger_(quill::create_logger("DataLogger")),
        queue_(1000), file_path_(file_path)
    {

        LOG_INFO(logger_, "Data is written to {}", file_path_);

        std::ifstream check_file(file_path_);
        bool file_exists = check_file.good();
        check_file.close();

        // Open the file in append mode
        std::ofstream file(file_path_, std::ios::app);

        if (file_exists) {
            // If the file exists, truncate it (delete all content)
            LOG_INFO(logger_, "File exists! Overwriting!");
            file.close();
            file.open(file_path_, std::ios::trunc | std::ios::out);
        }
        if (!file.is_open()) {
            LOG_ERROR(logger_, "Failed to open file: {}", file_path);
            return;
        }
        file_ = std::move(file);

        for (auto& val : WheelbotStateData::Headings ){
            log(val);
            if ( &val != &WheelbotStateData::Headings.back() ) log(",");
        }
        endl();

    }

    ~DataLogger(){
        file_.close();
        LOG_INFO(logger_, "Data was written to {}", file_path_);
    }

    inline auto get_lines_written(){return lines_written_;};


    template <typename T>
    void log(const T val){
        if( file_.is_open()){
            file_ << val;
        }
        else{
            LOG_ERROR_LIMIT(std::chrono::seconds(1), logger_, "Could not write to file {}! File not open!", file_path_);
        }
    }

    void endl(){
        if( file_.is_open()){
            file_ << std::endl;
        }
        else{
            LOG_ERROR_LIMIT(std::chrono::seconds(1), logger_, "Could not write to file {}! File not open!", file_path_);
        }
    }

    void log_eigen(const Eigen::MatrixXf& vec){
        // const Eigen::IOFormat CSVFormat(6, 0, ", ", "\n");
        const Eigen::IOFormat CSVFormat(6, Eigen::DontAlignCols, ", ", ", ", "", "");
        if(file_.is_open()){
            file_ << vec.format(CSVFormat);
        }
        else{
            LOG_ERROR_LIMIT(std::chrono::seconds(1), logger_, "Could not write to file {}! File not open!", file_path_);
        }
    }

    bool emplace_data(const WheelbotStateData::Data& dat) noexcept {
        return queue_.try_emplace(dat);
    }

    int log_data(){
        LOG_DEBUG(logger_, "Log Data");
        WheelbotStateData::Data dat;
        while(queue_.try_dequeue(dat)){
            log(double{dat.time/1000}/1.e6); log(",");
            log_eigen(dat.gyro0);           log(",");
            log_eigen(dat.gyro1);           log(",");
            log_eigen(dat.gyro2);           log(",");
            log_eigen(dat.gyro3);           log(",");
            log_eigen(dat.accel0);          log(",");
            log_eigen(dat.accel1);          log(",");
            log_eigen(dat.accel2);          log(",");
            log_eigen(dat.accel3);          log(",");
            log_eigen(dat.q_yrp);           log(",");
            log_eigen(dat.dq_yrp);          log(",");
            log_eigen(dat.q_DR);            log(",");
            log_eigen(dat.dq_DR);           log(",");
            log_eigen(dat.ddq_DR);          log(",");
            log_eigen(dat.tau_DR_command);  log(",");
            log_eigen(dat.setpoint_yrp);
            endl();
            lines_written_++;
            if(lines_written_ % 1000 == 0){
                LOG_DEBUG(logger_, "Lines written {}", lines_written_);
            }
        }
    return lines_written_;
    }
};


class DataLoggerThread : public cactus_rt::Thread {
    size_t                      max_loop_counter_;
    size_t                      loop_counter_ = 0;
    int64_t                     period_ns_ = 50'000'000;

    std::ofstream file_;

public:
    DataLoggerThread(std::string path) :
    Thread("Logger-thread", cactus_rt::ThreadConfig()),
    data_logger_(path, WheelbotStateData::Names)
    {
    }

    void Run() noexcept final {
        LOG_DEBUG(Logger(), "Entered Logger Run");
        while(true){
            loop_counter_++;
            data_logger_.log_data();

            if (this->StopRequested()) break;
            std::this_thread::sleep_for(std::chrono::nanoseconds(period_ns_));
        }
    }

    DataLogger data_logger_;
};

#endif // DATALOGGER_HPP
