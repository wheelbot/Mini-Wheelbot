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

#ifndef INPUTSERVER_HPP
#define INPUTSERVER_HPP

#include <iostream>
#include <boost/asio.hpp>
#include <nlohmann/json.hpp>
#include <optional>

#include <quill/Quill.h>
#include <unordered_map>
#include <cactus_rt/rt.h>

#include <atomic>
#include <memory>


using boost::asio::ip::tcp;
using json = nlohmann::json;


struct WheelbotInputCommand{
    float yaw_delta = 0;
    float drive_delta = 0;
};

struct WheelbotOdometryState{
    float position_x = 0;
    float position_y = 0;
    float velocity = 0;
    float orientation_yaw = 0;
};

class TcpServer : public std::enable_shared_from_this<TcpServer> {
    boost::asio::io_context io_context_;
    tcp::acceptor acceptor_;
    moodycamel::ReaderWriterQueue<WheelbotInputCommand> queue_command_;
    std::atomic<WheelbotOdometryState> latest_odometry_;


    quill::Logger* logger_;

public:
    inline
    boost::asio::io_context& get_io_context(){return io_context_;};

    void
    set_odometry_state(const WheelbotOdometryState& state) noexcept {
        latest_odometry_.store(state, std::memory_order_relaxed);
    }

    bool do_read(tcp::socket& socket){
        try {
            boost::asio::streambuf buffer;
            boost::system::error_code error;
            size_t len = boost::asio::read_until(socket, buffer, "\n", error);
            LOG_DEBUG(logger_, "Received len: {}", len);
            if (!error) {
                // Convert buffer to string
                std::istream input_stream(&buffer);
                std::string json_string;
                std::getline(input_stream, json_string);
                LOG_DEBUG(logger_, "Received string: {}", json_string);

                // Parse the JSON
                if (!json_string.empty()) {
                    json received_json = json::parse(json_string);
                    const float steering = received_json["yaw_delta"].get<float>();
                    const float velocity = received_json["drive_delta"].get<float>();

                    // Output the parsed values
                    LOG_DEBUG(logger_, "Received JSON: {}", received_json.dump());
                    LOG_DEBUG(logger_, "Steering: {}, Velocity: {}", steering, velocity);

                    queue_command_.try_emplace(WheelbotInputCommand{steering, velocity});
                }
                return true;
            } else if (error == boost::asio::error::eof) {
                LOG_INFO(logger_, "Client closed the connection.");
                return false;
            } else {
                LOG_ERROR(logger_, "Exception: {}", error.message());
                return false;
            }
        }
        catch (std::exception& e) {
            LOG_ERROR(logger_, "Exception: {}", e.what());
            return false;
        }
    }

    bool send_json(tcp::socket& socket) noexcept {
        try {
            WheelbotOdometryState odometry_state = latest_odometry_.load(std::memory_order_relaxed);

            json data = {
                {"position_x", odometry_state.position_x},
                {"position_y", odometry_state.position_y},
                {"velocity", odometry_state.velocity},
                {"orientation_yaw", odometry_state.orientation_yaw}
            };

            std::string json_string = data.dump() + "\n"; // Add newline for easy parsing on the client side
            boost::asio::write(socket, boost::asio::buffer(json_string));
            LOG_DEBUG(logger_, "Sent JSON: {}", json_string);
            return true;
        } catch (std::exception& e) {
            LOG_ERROR(logger_, "Failed to send JSON: {}", e.what());
            return false;
        }
    }

    void do_accept(std::function<void(tcp::socket&&)> on_accept) {
        // Create a new socket for the incoming connection
        auto socket = std::make_shared<tcp::socket>(io_context_);

        acceptor_.async_accept(*socket,
            [this, socket, on_accept](const boost::system::error_code& error) {
                if (!error) {
                    LOG_INFO(logger_, "New connection accepted.");
                    on_accept(std::move(*socket)); // Call the provided callback with the new socket
                } else {
                    LOG_ERROR(logger_, "Accept failed: {}", error.message());
                }
            }
        );
    }

    void start_accepting(std::function<bool()> stop_requested) {
        do_accept([this, stop_requested](tcp::socket&& new_socket) {
            LOG_INFO(logger_, "Handling new connection.");
            this->handle_connection(std::move(new_socket), stop_requested); // Process the new connection
        });
    }

    void handle_connection(tcp::socket&& socket, std::function<bool()> stop_requested) {
        LOG_INFO(logger_, "Connection established with client.");
        try {
            while (!stop_requested()) {
                if (!do_read(socket)) {
                    LOG_INFO(logger_, "Client disconnected or read failed. Exiting connection loop.");
                    break;
                }

                if (!send_json(socket)) {
                    LOG_WARNING(logger_, "Failed to send JSON data. Exiting connection loop.");
                    break; // Exit the loop if sending data fails
                }
            }
        } catch (const std::exception& e) {
            LOG_ERROR(logger_, "Exception in handle_connection: {}", e.what());
        }

        LOG_INFO(logger_, "Connection closed.");
    }



    TcpServer(uint16_t port)
        : acceptor_(io_context_, tcp::endpoint(tcp::v4(), port)),
        queue_command_(1000),
        logger_(quill::create_logger("DataLogger"))
    {
    }


    std::optional<WheelbotInputCommand> retrieve_new_command() noexcept {
        WheelbotInputCommand dat;
        if (queue_command_.try_dequeue(dat)) return std::make_optional<WheelbotInputCommand>(dat);
        else return std::nullopt;
    }
};

class InputServerThread : public cactus_rt::Thread {
    size_t                      max_loop_counter_;
    size_t                      loop_counter_ = 0;
    int64_t                     period_ns_ = 50'000'000;


public:
    InputServerThread(uint16_t port = 8888) :
    Thread("InputServer-thread", cactus_rt::ThreadConfig()),
    tcp_server_(port)
    {
    }

    void Run() noexcept final {
        while (!this->StopRequested()) {
            auto stop_requested = [this]() {
                return this->StopRequested();
            };

            tcp_server_.start_accepting(stop_requested);

            while (!this->StopRequested()) {
                tcp_server_.get_io_context().poll();
                if (!tcp_server_.get_io_context().stopped()) {
                    break;
                }
                std::this_thread::sleep_for(std::chrono::nanoseconds(period_ns_));
            }
        }
    }

    TcpServer tcp_server_;

};

#endif // INPUTSERVER_HPP
