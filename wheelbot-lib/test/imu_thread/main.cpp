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

#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <memory>
#include <poll.h>

#include "wheelbot/constants.hpp"
#include "wheelbot/gyro.hpp"
#include "wheelbot/accel.hpp"

#include <quill/Quill.h>
#include <cactus_rt/rt.h>

class ControllerThread : public cactus_rt::CyclicThread {
    std::array<std::shared_ptr<GyroThread>,4> gyroscopes_;
    std::array<std::shared_ptr<AccelThread>,4> accelerometer_;
    size_t                      max_loop_counter_;
    size_t                      loop_counter_ = 0;

    static cactus_rt::CyclicThreadConfig MakeRealTimeThreadConfig() {
        cactus_rt::CyclicThreadConfig config;
        config.period_ns = 1'000'000;
        config.cpu_affinity = std::vector<size_t>{3};
        config.SetFifoScheduler(99);

        return config;
    }
public:
    ControllerThread(
        const std::array<std::shared_ptr<GyroThread>,4>& gyroscopes,
        const std::array<std::shared_ptr<AccelThread>,4>& accelerometer)
        : CyclicThread("ControllerThread", MakeRealTimeThreadConfig()),
        gyroscopes_(gyroscopes),
        accelerometer_(accelerometer) {
    }

    int64_t GetLoopCounter() const {
        return loop_counter_;
    }

    bool Loop(int64_t ellapsed_ns) noexcept {
        // const double ellapsed_ms = static_cast<double>(ellapsed_ns) / 1'000'000.0;

        Eigen::Matrix<float, 3, 4> m;
        for (size_t i=0; i<4; i++){
            m(Eigen::all, i) = gyroscopes_.at(i)->gyro.get_latest_converted_measurement();
        }

        Eigen::Matrix<float, 3, 4> a;
        for (size_t i=0; i<4; i++){
            a(Eigen::all, i) = accelerometer_.at(i)->accel.get_latest_filtered_converted_measurement();
        }

        loop_counter_++;
        if (loop_counter_ % 1000 == 0) {
            LOG_INFO(Logger(), "Loop {}\n", loop_counter_);
            Eigen::IOFormat fmt(3,0, " ", "\n", "\t\t\t\t");
            std::stringstream ss_m;
            ss_m << m.format(fmt);
            LOG_INFO(Logger(), "m = \n{}", ss_m.str());
            std::stringstream ss_a;
            ss_a << a.format(fmt);
            LOG_INFO(Logger(), "a = \n{}", ss_a.str());
        }
        return false;
    }
};

int main(int argc, char * argv[])
{
    cactus_rt::App app;

    auto logger = quill::get_logger();
    LOG_INFO(logger,"IMU THREAD TEST");
    logger->set_log_level(quill::LogLevel::Debug);

    // auto g = GyroThread("gyro0");

    // auto g = std::make_shared<GyroThread>("gyro0");
    // auto a = std::make_shared<AccelThread>("acc0");

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
    auto controller_thread = std::make_shared<ControllerThread>(gyro_threads, acc_threads);

    app.StartTraceSession("build/data.perfetto");
    for ( const auto& t : gyro_threads ){
        app.RegisterThread(t);
    }
    for ( const auto& t : acc_threads ){
        app.RegisterThread(t);
    }
    app.RegisterThread(controller_thread);

    // constexpr unsigned int time = 5;
    // LOG_INFO(logger,"Testing RT loop for {} seconds.", time);

    // Sets up the signal handlers for SIGINT and SIGTERM (by default).
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
