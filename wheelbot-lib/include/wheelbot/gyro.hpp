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

#ifndef GYRO_HPP
#define GYRO_HPP

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


class GyroDevice{
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

    GyroDevice(
        const std::string devicename,
        const unsigned int sampling_frequency_value,
        const unsigned int low_pass_3db_to_value,
        const Eigen::Matrix3f& R = Eigen::Matrix3f::Identity()
        )
        : devicename_(devicename),
        logger_(quill::create_logger(devicename_+"-device")),
        sampling_frequency_value_(sampling_frequency_value),
        queue_(100),
        R_(R)
    {
        const std::array<std::string, 3> channel_names {"anglvel_x", "anglvel_y", "anglvel_z"};

        #ifdef HARDWARE_BUILD
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

        // {
        //     const std::string attribute_name{"sampling_frequency"};
        //     const auto attribute_value = std::to_string(sampling_frequency_value);
        //     {
        //         auto attr =  iio_channel_find_attr(channels.at(0), attribute_name.c_str());
        //         if (attr == NULL)
        //         {
        //             LOG_ERROR(logger_, "Failed to get attribute {}...\n{}", attribute_name, std::strerror(errno));
        //         }
        //         else{
        //             iio_channel_attr_write(channels.at(0), attr, attribute_value.c_str());
        //         }
        //     }
        // }

        {
            const std::string attribute_name{"filter_low_pass_3db_frequency"};
            const auto attribute_value = std::to_string(low_pass_3db_to_value);;
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
        }

        for (auto& c : channels){
            iio_channel_enable(c);
        }


        const std::string id =  iio_device_get_id(device);
        const std::string name =  iio_device_get_name(device);
        const std::string trigger_name = name + std::string{"-dev"}+id.back();
        LOG_INFO(logger_, "Trigger name: {}", trigger_name);
        trigger = iio_context_find_device(context, trigger_name.c_str());
        if (!trigger || !iio_device_is_trigger(trigger)) {
	    	LOG_ERROR(logger_, "No trigger found!\n{}",  std::strerror(errno));
    	}

        LOG_INFO(logger_, "Enabling IIO buffer trigger");
        if (iio_device_set_trigger(device, trigger)) {
            LOG_ERROR(logger_,"Could not set trigger\n{}",  std::strerror(errno));
        }

        buffer = iio_device_create_buffer(device, 1, false);

        if (buffer == NULL)
        {
            LOG_ERROR(logger_,"Failed to create imu buffer!\n{}",  std::strerror(errno));
        }

        // retrieve scaling
        for (size_t i=0; i<3; i++){
            const struct iio_data_format *fmt = iio_channel_get_data_format(channels.at(i));
            scale(i) = fmt->with_scale ? fmt->scale : 1;
        }
        #endif
    }

    ~GyroDevice(){
      #ifdef HARDWARE_BUILD
        if (buffer)
            iio_buffer_destroy(buffer);
        for (size_t i = 0; i<3; i++){
            if (channels.at(i))
                iio_channel_disable(channels.at(i));
        }

        if (device) {
		    int ret = iio_device_set_trigger(device, NULL);
            if (ret < 0) {
                char buf[256];
                iio_strerror(-ret, buf, sizeof(buf));
                LOG_ERROR(logger_, "%s while Disassociate trigger\n", buf);
            }
        }
        if (context)
            iio_context_destroy(context);
      #endif
    }

    void
    buffer_refill_blocking(){
        #ifdef HARDWARE_BUILD
        if (iio_buffer_refill(buffer) > 0)
            {
                ImuData dat;
                iio_channel_read(channels.at(0), buffer, &dat.raw_x, sizeof(int16_t));
                iio_channel_read(channels.at(1), buffer, &dat.raw_y, sizeof(int16_t));
                iio_channel_read(channels.at(2), buffer, &dat.raw_z, sizeof(int16_t));
                queue_.try_enqueue(dat);
            }
        #endif
    }

    Eigen::Vector3f get_latest_converted_measurement(){
        #ifdef HARDWARE_BUILD
        ImuData dat;
        while (queue_.try_dequeue(dat)) {
            Eigen::Vector3f m{
                static_cast<float>(dat.raw_x),
                static_cast<float>(dat.raw_y),
                static_cast<float>(dat.raw_z) };
            last_converted_measurement = R_*m.cwiseProduct(scale);
        }
        return last_converted_measurement;
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
    struct iio_buffer *buffer;

    std::array<struct iio_channel *, 3> channels;
    std::array<int16_t, 3> raw_value;

    Eigen::Vector3f scale{1,1,1};
    Eigen::Vector3f last_converted_measurement{0,0,0};
    const Eigen::Matrix3f R_;


};


/**
 * This is a no-op thread that does nothing at 1 kHz.
 */
class GyroThread : public cactus_rt::Thread {
    int64_t loop_counter_ = 0;
    uint64_t        period_ns_;


    static cactus_rt::CyclicThreadConfig MakeRealTimeThreadConfig() {
        cactus_rt::CyclicThreadConfig config;
        config.period_ns = 1'000'000;
        config.cpu_affinity = std::vector<size_t>{1};
        config.SetFifoScheduler(90);

        return config;
    }

 public:
  GyroThread(std::string name = "gyro0",
        Eigen::Matrix3f R = Eigen::Matrix3f::Identity(),
        cactus_rt::CyclicThreadConfig config =  MakeRealTimeThreadConfig())
    : Thread(name+"-thread", config), gyro(GyroDevice(name, 1000, 116, R)), period_ns_(config.period_ns) {
    }

  int64_t GetLoopCounter() const {
    return loop_counter_;
  }

    GyroDevice gyro;

 protected:
  void Run() noexcept final {
    int64_t loop_start, loop_end;
    while (!this->StopRequested()){
      loop_start = cactus_rt::NowNs();
      gyro.buffer_refill_blocking();
      loop_end = cactus_rt::NowNs();
      loop_counter_++;
      if (loop_counter_ % 1000 == 0) {
        LOG_DEBUG(this->Logger(), "Loop {}", loop_counter_);
      }
    //   if ( static_cast<uint64_t>(loop_end - loop_start) >= this->period_ns_ ) {
    //   LOG_TRACE_L3_LIMIT(
    //     std::chrono::milliseconds(100),
    //     this->Logger(),
    //     "At least 1 loop overrun detected in the last 100ms: latency ({}ns) > period ({}ns)",
    //     loop_end - loop_start,
    //     period_ns_
    //   );
    //   }
    }
  }

  Eigen::Vector3f get_measurement(){
    #ifdef HARDWARE_BUILD
    return gyro.get_latest_converted_measurement();
    #else
    Eigen::Vector3f ret;
    ret.setZero();
    return ret;
    #endif
  }

};

#endif // GYRO_HPP
