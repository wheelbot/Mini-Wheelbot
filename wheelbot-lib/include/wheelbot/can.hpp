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

#ifndef CAN_HPP
#define CAN_HPP

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <numbers>

#include <readerwriterqueue.h>
#include <eigen3/Eigen/Dense>
#include <quill/Quill.h>
#include <cactus_rt/rt.h>


long long int
tvToTimestamp(const timeval& tv){
    return tv.tv_sec*1'000'000'000+tv.tv_usec*1'000;
}

constexpr std::array<int, 2> motor_sign{1,-1};

class DataFromMotor
{
public:
    void
    updateFromMessageData(const can_frame& frame){
        motor_id = frame.can_id - BASE_ID;
        angle    = motor_sign.at(motor_id-1)*static_cast<float>(int16_t{(frame.data[0]<<8) | frame.data[1]}) / 100.0 / 360.0 * 2.0 * std::numbers::pi;
        // velocity = static_cast<float>((int16_t{frame.data[2]}<<8) | int16_t{frame.data[3]}) / 10.f;
        // const auto current  = static_cast<float>((int16_t{frame.data[4]}<<8) | int16_t{frame.data[5]}) / 1000.f;
        // torque = current * 0.5 / 20.0;
    }

    void
    updateFromMessageData(const timeval& tv, const can_frame& frame){
        receive_timestamp = tvToTimestamp(tv);
        updateFromMessageData(frame);
    }

    static constexpr uint8_t CAN_FRAME_LEN = 2;
    static constexpr uint8_t BASE_ID = 0x80;

    uint8_t motor_id;
    float angle;
    long long int receive_timestamp;
    long long int sync_timestamp;
    // float velocity;
    // float torque;
};

class DataToMotor
{
public:
    DataToMotor(){
        torque_ref = 0;
    }

    // DataToMotor(uint8_t motor_id, const float gain = 0, const float velocity_ref = 0, const float torque_ref = 0)
    DataToMotor(uint8_t motor_id, const float torque_ref = 0)
    : motor_id(motor_id), torque_ref(torque_ref)
    {
    }

    auto
    toMessageData(){
        struct can_frame frame;
        frame.len = CAN_FRAME_LEN;
        frame.can_id = BASE_ID + motor_id;

        // const int16_t K_fp = static_cast<int16_t>(gain);
		// frame.data[0] = K_fp >> 8;
		// frame.data[1] = K_fp & 0xff;

		// const int16_t v_ref_fp = static_cast<int16_t>(velocity_ref);
		// frame.data[2] = v_ref_fp >> 8;
		// frame.data[3] = v_ref_fp & 0xff;

        const auto current_ref = motor_sign.at(motor_id-1)*torque_ref * 20.0 / 0.5;
		const int16_t u_fp = static_cast<int16_t>(current_ref*1000.f);
		frame.data[0] = u_fp >> 8;
		frame.data[1] = u_fp & 0xff;

        return frame;
    }

    static constexpr uint8_t CAN_FRAME_LEN = 2;
    static constexpr uint8_t BASE_ID = 0x10;

    uint8_t motor_id;
    // float gain;
    // float velocity_ref;
    float torque_ref;
};

class Sync
{
public:
    static constexpr uint8_t CAN_FRAME_LEN = 0;
    static constexpr uint8_t BASE_ID = 0x0;

    auto
    toMessageData(){
        struct can_frame empty_frame;
        empty_frame.can_id = BASE_ID;
        empty_frame.len = CAN_FRAME_LEN;
        return empty_frame;
    }
};


bool
isSameMessage(const uint8_t id, const uint8_t other_id){
    return ( id >> 4 ) == ( other_id >> 4 );
}

using CanMessage = std::variant<std::monostate, DataFromMotor>;

CanMessage
receiveCanMessage(const std::pair<timeval, can_frame>& msg){
        const auto& tv = std::get<0>(msg);
        const auto& frame = std::get<1>(msg);
        if ( isSameMessage(frame.can_id, DataFromMotor::BASE_ID) &&
             ( frame.len == DataFromMotor::CAN_FRAME_LEN) )
        {
            DataFromMotor dat;
            dat.updateFromMessageData(tv, frame);
            return dat;
        }
        return std::monostate{};
}

CanMessage
receiveCanMessage(const std::optional<std::pair<timeval, can_frame>>& msg){
        if (msg.has_value()) return receiveCanMessage(msg.value());
        return std::monostate{};
}


class Can{
public:
void
    initCanSocket(const std::string interface_name = "motorCan" ){
        #ifdef HARDWARE_BUILD
        const char *interfaceName = interface_name.c_str();
        this->sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock < 0) {
            perror("Socket creation error");
            return;
        }
        int priority = 0;

        if (setsockopt(sock, SOL_SOCKET, SO_PRIORITY, &priority, sizeof(priority)) == -1) {
            std::cerr << "Error setting socket option SO_PRIORITY" << std::endl;
            return;
        }

        int loopback = 0; /* 0 = disabled, 1 = enabled (default) */
        setsockopt(sock, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));


        strcpy(this->ifr.ifr_name, interfaceName);
        ioctl(this->sock, SIOCGIFINDEX, &ifr);
        this->addr.can_family = AF_CAN;
        this->addr.can_ifindex = this->ifr.ifr_ifindex;


        if (bind(this->sock, (struct sockaddr *)&(this->addr), sizeof(this->addr)) < 0) {
            perror("Bind error");
            return;
        }

        const int timestamp_on = 1;
        if (setsockopt(this->sock, SOL_SOCKET, SO_TIMESTAMP,
                    &timestamp_on, sizeof(timestamp_on)) < 0) {
            perror("setsockopt SO_TIMESTAMP");
            return;
        }

        // set can filter
        // if (this->filter)
        setsockopt(this->sock, SOL_CAN_RAW, CAN_RAW_FILTER,
            &(this->filter), sizeof this->filter);

        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        setsockopt(this->sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
        #endif
    }

    void sendCanFrame(const struct can_frame frame){
        // if (write(this->sock, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            // perror("Write error");
            // return;
        // }
        #ifdef HARDWARE_BUILD
        if (sendto(this->sock, &frame, sizeof(struct can_frame), MSG_DONTWAIT,
                (struct sockaddr *)&addr, sizeof(addr)) != sizeof(struct can_frame)) {
            perror("Write error");
            return;
        }
        #endif
        return;
    }

    std::optional<std::pair<timeval, can_frame>>
    receiveCanFrame(){
        struct msghdr msg;
	    struct cmsghdr *cmsg;
        struct iovec iov;
        struct can_frame frame;
        struct timeval tv;
        char ctrlmsg[CMSG_SPACE(sizeof(struct timeval)) +
		     CMSG_SPACE(3 * sizeof(struct timespec)) +
		     CMSG_SPACE(sizeof(__u32))];
        iov.iov_base = &frame;
	    msg.msg_name = &(this->addr);
	    msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_control = &ctrlmsg;

        msg.msg_namelen = sizeof(this->addr);
        msg.msg_controllen = sizeof(ctrlmsg);
        msg.msg_flags = 0;

        iov.iov_len = sizeof(frame);
        msg.msg_namelen = sizeof(addr);
        msg.msg_controllen = sizeof(ctrlmsg);
        msg.msg_flags = 0;

        ssize_t nbytes = recvmsg(this->sock, &msg, 0);
        // ssize_t nbytes = recv(this->sock, &receivedFrame, sizeof(struct can_frame), 0);


        if (nbytes < 0) {
            perror("Read error WTF");
            return std::nullopt;
        } else if (nbytes < ssize_t{sizeof(struct can_frame)}) {
            std::cerr << "Incomplete CAN frame received" << std::endl;
            return std::nullopt;
        }

        // std::cout << "Received CAN frame with ID: 0x" << std::hex << receivedFrame.can_id << std::dec << std::endl;


        for (cmsg = CMSG_FIRSTHDR(&msg);
                cmsg && (cmsg->cmsg_level == SOL_SOCKET);
                cmsg = CMSG_NXTHDR(&msg,cmsg)) {
            if (cmsg->cmsg_type == SO_TIMESTAMP)
                memcpy(&tv, CMSG_DATA(cmsg), sizeof(tv));
        }


        // std::cout << "Got Message " << frame.can_id << " tv " << tv.tv_sec << std::endl;

        return std::make_optional<std::pair<timeval, can_frame>>(std::make_pair(tv, frame));
    }

    void
    closeCanSocket(){
        if ( (this->sock) > 0 )
            close(this->sock);
    }

    ~Can(){
        closeCanSocket();
    }

    Can(const can_filter f = {}){
        filter = f;
        initCanSocket();
    }

private:
    std::string can_interface_name;
    int sock;

    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_filter filter;
    size_t filter_len;
};



template<size_t N_MOTORS=2>
class MotorInterface{
public:
    std::array<DataToMotor, N_MOTORS> data_to_motors;
    std::array<DataFromMotor, N_MOTORS> data_from_motors;

    Can sendSock;
    Can receiveSock;
    moodycamel::ReaderWriterQueue<DataFromMotor> data_from_motor_queue_;
    std::array<moodycamel::ReaderWriterQueue<unsigned long long int>, N_MOTORS> sync_timestamp_queue_;


    MotorInterface()
        :
        sendSock(Can()),
        receiveSock(Can()){}

    // Eigen::Matrix<double, 3, N_MOTORS>
    // get_motor_state(){
    //     send_sync();
    //     receive_motor_state();
    // }

    void
    send_sync(){
        #ifdef HARDWARE_BUILD
        sendSock.sendCanFrame(Sync().toMessageData());
        auto time = cactus_rt::NowNs();
        for (int i=0; i<N_MOTORS; i++){
            sync_timestamp_queue_.at(i).try_enqueue(time);
        }
        #endif
    }

    std::optional<DataFromMotor>
    try_get_motor_state(){
        DataFromMotor dat;
        return data_from_motor_queue_.try_dequeue(dat) ? std::make_optional<DataFromMotor>(dat) : std::nullopt;
    }

    void
    receive_can_message(){
        #ifdef HARDWARE_BUILD
        std::visit(overloaded{
            [](std::monostate) {}, // No message
            [&](DataFromMotor dat){
                sync_timestamp_queue_.at(dat.motor_id-1).try_dequeue(dat.sync_timestamp);
                data_from_motor_queue_.try_enqueue(dat);}
        },
        receiveCanMessage(receiveSock.receiveCanFrame()));
        #endif
    }

    template<class... Ts>
    struct overloaded : Ts... { using Ts::operator()...; };

    void send_motor_command(Eigen::Vector<float, N_MOTORS> u){
        #ifdef HARDWARE_BUILD
        for (size_t i=0; i<N_MOTORS; i++){
            data_to_motors.at(i).torque_ref = u(i);
            data_to_motors.at(i).motor_id = static_cast<uint8_t>(i+1);
            sendSock.sendCanFrame(data_to_motors.at(i).toMessageData());
        }
        #endif
    }
};

class MotorInterfaceThread : public cactus_rt::Thread {

    static cactus_rt::CyclicThreadConfig MakeRealTimeThreadConfig() {
        cactus_rt::CyclicThreadConfig config;
        config.period_ns = 1'000'000;
        config.cpu_affinity = std::vector<size_t>{3};
        config.SetFifoScheduler(90);
        return config;
    }

public:
    MotorInterfaceThread(
        std::string name = "motor",
        cactus_rt::CyclicThreadConfig config = MakeRealTimeThreadConfig())
        :
        motor(MotorInterface<>()),
        Thread(name+"-thread", config){}

    MotorInterface<> motor;

protected:
    void Run() noexcept final {
        while (!this->StopRequested()){
            #ifdef HARDWARE_BUILD
            motor.receive_can_message();
            #endif
            // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

};

#endif
