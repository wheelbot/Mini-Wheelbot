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

#include "wheelbot/can.hpp"


#include <quill/Quill.h>
#include <cactus_rt/rt.h>


static constexpr unsigned int SAMPLES = 1000;

/**
 * This is a no-op thread that does nothing at 1 kHz.
 */
class ExampleRTThread : public cactus_rt::CyclicThread {
  int64_t loop_counter_ = 0;
  int64_t tocs_counter_ = 0;
  std::shared_ptr<MotorInterfaceThread> motor_;
  std::array<int64_t, SAMPLES> tics;
  std::array<int64_t, SAMPLES> tocs;

  char *chipname = "gpiochip0";
	unsigned int line_num = 13;	// GPIO Pin #23
	unsigned int val;
	struct gpiod_chip *chip;
	struct gpiod_line *line;
	int i, ret;

 public:
  ExampleRTThread(
    std::shared_ptr<MotorInterfaceThread> motor,
    const char* name,
    cactus_rt::CyclicThreadConfig config)
    : motor_(motor),
      cactus_rt::CyclicThread(name, config) {
  }

  int64_t GetLoopCounter() const {
    return loop_counter_;
  }

 double mean_tocs(){
    double sum = 0;
    for (const auto& t : tocs){
        sum += (static_cast<double>(t)/tocs.size());
    }
    return sum/1000000.0;
 }

 double min_tocs(){
    auto min = tocs.at(0);
    for (const auto& t : tocs){
        if ( t < min) min = t;
    }
    return static_cast<double>(min)/1'000'000.0;
 }

 double max_tocs(){
    auto max = tocs.at(0);
    for (const auto& t : tocs){
        if ( t > max) max = t;
    }
    return static_cast<double>(max)/1'000'000.0;

 }

 protected:
  bool Loop(int64_t /*now*/) noexcept final {
    if (loop_counter_ < SAMPLES){
      tics.at(loop_counter_) = cactus_rt::NowNs();
      motor_->motor.send_sync();
      // if ( tocs.at(loop_counter_) > 2'000'000 )
        // LOG_INFO(Logger(), "Problem loop {}", loop_counter_);

      if (loop_counter_ % 1000 == 0) {
        LOG_INFO(Logger(), "Loop {}", loop_counter_);
      }
      loop_counter_++;
    }

    if (tocs_counter_ < SAMPLES){
      auto res = motor_->motor.try_get_motor_state();
      if ( res.has_value() && ( res.value().motor_id == 1 ) ){
        tocs.at(tocs_counter_) = res.value().sync_timestamp-tics.at(tocs_counter_);
        LOG_INFO(Logger(), "Angle {}", res.value().angle);
        tocs_counter_++;
      }
    }

    if ((loop_counter_ >= SAMPLES) && (tocs_counter_ >= SAMPLES)) return true;
    // LOG_INFO(Logger(), "tics {}, tocs {}", loop_counter_, tocs_counter_);

    return false;
  }
};

int main() {
  cactus_rt::CyclicThreadConfig config;
  config.period_ns = 1'000'000;
  config.cpu_affinity = std::vector<size_t>{2};
  config.SetFifoScheduler(90);

  constexpr unsigned int time = SAMPLES/1000+1;
  auto motor_thread = std::make_shared<MotorInterfaceThread>("motor");
  auto main_thread = std::make_shared<ExampleRTThread>(motor_thread, "ExampleRTThread", config);

  cactus_rt::App app;
  app.StartTraceSession("build/data.perfetto");
  app.RegisterThread(motor_thread);
  app.RegisterThread(main_thread);

  std::cout << "Testing RT loop for " << time << " seconds.\n";

  app.Start();
  std::this_thread::sleep_for(std::chrono::seconds(time));
  app.RequestStop();
  app.Join();



  std::cout << "Number of loops executed: " << main_thread->GetLoopCounter() << "\n";
  std::cout << "Mean time: " << main_thread->mean_tocs()
    << "   Min time: " << main_thread->min_tocs()
    << "   Max time: " << main_thread->max_tocs() << "\n";
  return 0;
}
