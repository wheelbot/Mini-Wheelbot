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

#include <thread>
#include <chrono>

#include "wheelbot/accel.hpp"

#include <quill/Quill.h>

void setup_logger(){
    quill::Config cfg;
    cfg.enable_console_colours = true;
    quill::configure(cfg);

    quill::start();
}


int main(int argc, char * argv[])
{
    setup_logger();
    auto logger = quill::get_logger();
    LOG_INFO(logger, "IMU TEST");
    logger->set_log_level(quill::LogLevel::Debug);


    std::string device_name = "acc0";
    auto sleep_time = std::chrono::seconds(1);
    #ifdef HARDWARE_BUILD
    AccelDevice a0(device_name, 1000);
    LOG_INFO(logger, "Blocking read {} samples with {} seconds wait:", device_name, sleep_time);
    for (size_t i = 0; i<10; i++){
        a0.buffer_refill_blocking();
        LOG_INFO(logger, "rate = {}' rad/s", a0.get_latest_converted_measurement().transpose());
        std::this_thread::sleep_for(sleep_time);

    }
    #endif
}
