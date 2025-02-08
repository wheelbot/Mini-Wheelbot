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

#ifndef ACCEL_HPP
#define ACCEL_HPP

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <iio.h>
#include <assert.h>
#include <unistd.h>

#include <iostream>

#include <array>
#include <string>
#include <atomic>

#include <eigen3/Eigen/Dense>
#include <readerwriterqueue.h>
#include <quill/Quill.h>
#include <cactus_rt/rt.h>


template< typename T>
T IIRFilter(const T& x, const T& y, const double& alpha=0.1){
    return alpha*x + (1-alpha)*y;
}

class AccelDevice{
public:

    std::string devicename_;
    quill::Logger* logger_;
    unsigned int sampling_frequency_value_;

    struct ImuData {
        int16_t raw_x;
        int16_t raw_y;
        int16_t raw_z;
    };

    moodycamel::ReaderWriterQueue<ImuData> queue_;

    AccelDevice(
        const std::string devicename,
        const unsigned int sampling_frequency_value,
        const Eigen::Matrix3f& R = Eigen::Matrix3f::Identity())
        : devicename_(devicename),
        logger_(quill::create_logger(devicename_+"-device")),
        sampling_frequency_value_(sampling_frequency_value),
        queue_(100),
        R_(R)
    {
        #ifdef HARDWARE_BUILD
        const std::array<std::string, 3> channel_names {"accel_x", "accel_y", "accel_z"};;

        context =  iio_create_local_context();
        if (context == NULL)
        {
            LOG_ERROR(logger_, "Failed to acquire default context!\n{}", std::strerror(errno));
            return;
        }
        device = iio_context_find_device(context, devicename.c_str());
        if (device == NULL)
        {
            LOG_ERROR(logger_, "Failed find device {}!\n{}", devicename_, std::strerror(errno));
            return;
        }

        LOG_INFO(logger_, "Configuring Device ID: {}, Name: {}", iio_device_get_id(device), iio_device_get_name(device));

        for (size_t i = 0; i<3; i++){
            channels.at(i) = iio_device_find_channel(device, channel_names.at(i).c_str(), false);
            if (channels.at(i) == NULL)
            {
                LOG_ERROR(logger_, "Failed to get channel for {}...\n{}", channel_names.at(i), std::strerror(errno));
            }
        }

        const std::string attribute_name{"sampling_frequency"};
        const auto attribute_value = std::to_string(sampling_frequency_value);;
        {
            auto attr =  iio_channel_find_attr(channels.at(0), attribute_name.c_str());
            if (attr == NULL)
            {
                LOG_ERROR(logger_, "Failed to get attribute {}...\n{}", attribute_name, std::strerror(errno));
            }
            else{
                iio_channel_attr_write(channels.at(0), attr, attribute_value.c_str());
            }
        }

        for (auto& c : channels){
            iio_channel_enable(c);
        }

        // retrieve scaling
        for (size_t i=0; i<3; i++){
            const struct iio_data_format *fmt = iio_channel_get_data_format(channels.at(i));
            scale(i) = fmt->with_scale ? fmt->scale : 1;
        }

        // get one measurement to initialize filter
        buffer_refill_blocking();
        last_filtered_converted_measurement = get_latest_converted_measurement();
        #endif
    }

    ~AccelDevice(){
        for (size_t i = 0; i<3; i++){
            if (channels.at(i))
                iio_channel_disable(channels.at(i));
        }
        if (context)
            iio_context_destroy(context);
    }

    void
    buffer_refill_blocking(){
        #ifdef HARDWARE_BUILD
        ImuData dat;
        int ret = 0;
        ret = (int) iio_channel_attr_read(channels.at(0), "raw", buffer, buffer_length);
        dat.raw_x = ret > 0 ? std::stoi(std::string(buffer)) : 0;
        ret = (int) iio_channel_attr_read(channels.at(1), "raw", buffer, buffer_length);
        dat.raw_y = ret > 0 ? std::stoi(std::string(buffer)) : 0;
        ret = (int) iio_channel_attr_read(channels.at(2), "raw", buffer, buffer_length);
        dat.raw_z = ret > 0 ? std::stoi(std::string(buffer)) : 0;

        queue_.try_enqueue(dat);
        #endif
    }

    Eigen::Vector3f get_latest_converted_measurement(){
        #ifdef HARDWARE_BUILD
        ImuData dat;
        while (queue_.try_dequeue(dat)) {
            Eigen::Vector3f measurement{
                static_cast<float>(dat.raw_x),
                static_cast<float>(dat.raw_y),
                static_cast<float>(dat.raw_z) };
            Eigen::Vector3f last_converted_measurement = R_*measurement.cwiseProduct(scale/1000.*9.81); // from [milli-g] to [m/s^2]

        }
        return last_converted_measurement;
        #else
        Eigen::Vector3f ret;
        ret.setZero();
        return ret;
        #endif
    }

    Eigen::Vector3f get_latest_filtered_converted_measurement(){
        #ifdef HARDWARE_BUILD
        ImuData dat;
        while (queue_.try_dequeue(dat)) {
            Eigen::Vector3f measurement{
                static_cast<double>(dat.raw_x),
                static_cast<double>(dat.raw_y),
                static_cast<double>(dat.raw_z) };
            Eigen::Vector3f last_converted_measurement = R_*measurement.cwiseProduct(scale/1000.*9.81); // from [milli-g] to [m/s^2]
            last_filtered_converted_measurement = IIRFilter(last_converted_measurement, last_filtered_converted_measurement, 0.1);
            // last_filtered_converted_measurement = last_converted_measurement;
        }
        return last_filtered_converted_measurement;
        #else
        Eigen::Vector3f ret;
        ret.setZero();
        return ret;
        #endif
    }

    inline quill::Logger*         Logger() const { return logger_; }

    struct iio_context *context;
    struct iio_device *device;
    struct iio_device *trigger;

    const size_t buffer_length{100};
    char* buffer = new char[buffer_length];

    std::array<struct iio_channel *, 3> channels;
    std::array<unsigned int, 3> channels_raw_value_idx;
    std::array<int16_t, 3> raw_value;

    Eigen::Vector3f scale{1,1,1};
    Eigen::Vector3f last_converted_measurement{0,0,0};
    Eigen::Vector3f last_filtered_converted_measurement{0,0,0};

    const Eigen::Matrix3f R_;

    std::function<Eigen::Vector3f(Eigen::Vector3f,Eigen::Vector3f)> filter;
};

/**
 * This is a no-op thread that does nothing at 1 kHz.
 */
class AccelThread : public cactus_rt::CyclicThread {
    int64_t  loop_counter_ = 0;
    uint64_t period_ns_;

    static cactus_rt::CyclicThreadConfig MakeRealTimeThreadConfig() {
        cactus_rt::CyclicThreadConfig config;
        config.period_ns = 5'000'000;
        config.cpu_affinity = std::vector<size_t>{2};
        config.SetFifoScheduler(80);

        return config;
    }


 public:
  AccelThread(
    std::string name = "acc0",
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity(),
    cactus_rt::CyclicThreadConfig config =  MakeRealTimeThreadConfig())
    : CyclicThread(name+"-thread", config), accel(AccelDevice(name, 800, R)), period_ns_(config.period_ns) {
    }

  int64_t GetLoopCounter() const {
    return loop_counter_;
  }

    AccelDevice accel;


 protected:
  bool Loop(int64_t ellapsed_ns) noexcept final {
      accel.buffer_refill_blocking();
    loop_counter_++;
    if (loop_counter_ % 1000 == 0) {
        LOG_DEBUG(Logger(), "Loop {}", loop_counter_);
    }
      return false;
  }

  Eigen::Vector3f get_measurement(){
    return accel.get_latest_filtered_converted_measurement();
  }
};

#endif // ACCEL_HPP
