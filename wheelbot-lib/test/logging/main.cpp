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
#include <optional>

#include "wheelbot/datalogger.hpp"

#include <quill/Quill.h>
#include <cactus_rt/rt.h>

class ControllerThread : public cactus_rt::CyclicThread {
  std::shared_ptr<DataLoggerThread> data_logger_;
  size_t                      iterations_ = 0;
  std::optional<uint64_t>   starting_time;

  static cactus_rt::CyclicThreadConfig MakeRealTimeThreadConfig() {
    cactus_rt::CyclicThreadConfig config;
    config.period_ns = 1'000'000;
    config.cpu_affinity = std::vector<size_t>{2};
    config.SetFifoScheduler(80);

    return config;
  }

WheelbotStateData::Data data_;

 public:
  ControllerThread(std::shared_ptr<DataLoggerThread> data_logger)
      : CyclicThread("ControllerThread", MakeRealTimeThreadConfig()),
        data_logger_(data_logger) {
            starting_time = std::nullopt;
        LOG_INFO(Logger(), "Setting up controller thread");
  }

 protected:
  bool Loop(int64_t ellapsed_ns) noexcept final{
      if(!starting_time.has_value()){
          starting_time=cactus_rt::NowNs();
          data_.time = 0;
      }
      else{
          data_.time = cactus_rt::NowNs() - starting_time.value();
      }
      data_.gyro0 = Eigen::Vector3f::Random();
      data_.gyro1 = Eigen::Vector3f::Random();
      data_.gyro2 = Eigen::Vector3f::Random();
      data_.gyro3 = Eigen::Vector3f::Random();
      data_.accel0 = Eigen::Vector3f::Random();
      data_.accel1 = Eigen::Vector3f::Random();
      data_.accel2 = Eigen::Vector3f::Random();
      data_.accel3 = Eigen::Vector3f::Random();
      data_.q_yrp = Eigen::Vector3f::Random();
      data_.dq_yrp = Eigen::Vector3f::Random();
      data_.q_DR = Eigen::Vector2f::Random();
      data_.dq_DR = Eigen::Vector2f::Random();
      data_.tau_DR_command = Eigen::Vector2f::Random();
      data_logger_->data_logger_.emplace_data(data_);
      if (iterations_++ > 2) return true;
      return false;
  }

public:
  inline auto get_iterations(){return iterations_;};
};

int main(int argc, char * argv[])
{
    cactus_rt::App app;
    // quill::start();
    auto logger = quill::get_logger();
    LOG_INFO(logger,"DATA LOGGER TEST");
    logger->set_log_level(quill::LogLevel::Debug);
    quill::flush();

    #ifdef HARDWARE_BUILD
    auto logger_thread = std::make_shared<DataLoggerThread>("/wheelbot_lib/log");
    #else
    auto logger_thread = std::make_shared<DataLoggerThread>("../../log");
    #endif
    auto controller_thread = std::make_shared<ControllerThread>(logger_thread);

    app.RegisterThread( logger_thread );
    app.RegisterThread( controller_thread );

    constexpr unsigned int time = 5;
    LOG_INFO(logger,"Testing RT loop for {} seconds.", time);

    // Sets up the signal handlers for SIGINT and SIGTERM (by default).
    // cactus_rt::SetUpTerminationSignalHandler();
    // LOG_INFO(logger,"Testing RT loop until CTRL+C.");

    app.Start();
    LOG_INFO(logger,"App started");
    // // This function blocks until SIGINT or SIGTERM are received.
    // cactus_rt::WaitForAndHandleTerminationSignal();
    // LOG_INFO(logger,"App SIGINIT/SIGTERM received");
    std::this_thread::sleep_for(std::chrono::seconds(time));
    app.RequestStop();
    app.Join();

    LOG_INFO(logger, "Number of iterations {}, number of lines written {}", controller_thread->get_iterations(), logger_thread->data_logger_.get_lines_written());
    quill::flush();
    return 0;

}
